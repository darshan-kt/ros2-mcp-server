"""DefaultExecutionManager — structurally satisfies
ros_mcp.contracts.execution.ExecutionManager (docs/13-contracts.md §6,
docs/06-execution.md state machine).

This is the *only* public entry point (`submit`) that results in a MOTION/HIGH_RISK
adapter being invoked, and it always calls SafetyPolicyEngine.check() before any adapter
reference is dereferenced — the Non-Bypass Rule (docs/13-contracts.md §5) enforced here,
structurally, not by convention.
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Awaitable, Callable

from ros_mcp.contracts.capabilities import CapabilityRegistry
from ros_mcp.contracts.core import CommandClass, ExecutionState, Operation, Pose2D, SemanticCommand
from ros_mcp.contracts.errors import ErrorCode, ToolError, ToolResult
from ros_mcp.contracts.execution import CommandPlanner
from ros_mcp.contracts.results import StopResult
from ros_mcp.contracts.safety import SafetyDecision, SafetyPolicyEngine
from ros_mcp.execution.cancellation import SimpleCancellationToken
from ros_mcp.execution.error_results import build_error_result, build_result_with_defaults

logger = logging.getLogger(__name__)

# Backstop above the command's own timeout_s: the adapter is required to self-timeout
# (07-motion-architecture.md, 08-navigation-architecture.md) — this only fires if that
# internal enforcement somehow fails, per 06-execution.md ("does not trust adapters to
# self-timeout... defense in depth").
_TIMEOUT_BACKSTOP_MARGIN_S = 2.0

ReadHandler = Callable[[SemanticCommand], Awaitable[ToolResult]]


class DefaultExecutionManager:
    """Structurally satisfies ros_mcp.contracts.execution.ExecutionManager."""

    def __init__(
        self,
        *,
        registry: CapabilityRegistry,
        command_planner: CommandPlanner,
        safety_policy_engine: SafetyPolicyEngine,
        current_pose_provider: Callable[[], Awaitable[Pose2D | None]],
        read_handlers: dict[Operation, ReadHandler],
        motion_backend: Any = None,
        navigation_backend: Any = None,
    ) -> None:
        self._registry = registry
        self._command_planner = command_planner
        self._safety_policy_engine = safety_policy_engine
        self._current_pose_provider = current_pose_provider
        self._read_handlers = read_handlers
        self._motion_backend = motion_backend
        self._navigation_backend = navigation_backend

        self._in_flight: dict[str, "asyncio.Future[ToolResult]"] = {}
        self._tokens: dict[str, SimpleCancellationToken] = {}
        self._active_commands: dict[str, SemanticCommand] = {}  # robot_id -> command
        self._progress_callbacks: dict[str, list[Callable[[dict[str, Any]], None]]] = {}

    async def submit(self, command: SemanticCommand) -> ToolResult:
        existing = self._in_flight.get(command.command_id)
        if existing is not None:
            return await existing

        future: "asyncio.Future[ToolResult]" = asyncio.ensure_future(self._execute(command))
        self._in_flight[command.command_id] = future
        try:
            return await future
        finally:
            self._in_flight.pop(command.command_id, None)

    async def cancel(self, command_id: str, reason: str) -> bool:
        token = self._tokens.get(command_id)
        if token is None:
            return False
        token.cancel()
        logger.info("cancelled command %s: %s", command_id, reason)
        return True

    def active_command(self, robot_id: str) -> SemanticCommand | None:
        return self._active_commands.get(robot_id)

    def on_progress(
        self, command_id: str, callback: Callable[[dict[str, Any]], None]
    ) -> None:
        self._progress_callbacks.setdefault(command_id, []).append(callback)

    def set_read_handlers(self, read_handlers: dict[Operation, ReadHandler]) -> None:
        """Not part of the frozen ExecutionManager Protocol — a construction-time
        wiring seam for server.py, needed because building the read handlers
        (ros_mcp.mcp.read_handlers.build_read_handlers) itself requires a reference to
        this manager instance (for robot.get_state's active_command lookup), creating
        an unavoidable construction-order cycle resolved by constructing the manager
        with an empty dict first and populating it here immediately after."""
        self._read_handlers = read_handlers

    def emit_progress(self, command_id: str, payload: dict[str, Any]) -> None:
        """Not part of the frozen ExecutionManager Protocol — the write-side seam
        adapters (Nav2NavigationPlugin, step 6) use to push feedback through the
        callbacks registered via on_progress (11-context-and-streaming.md)."""
        for callback in self._progress_callbacks.get(command_id, ()):
            try:
                callback(payload)
            except Exception:  # noqa: BLE001 - one bad subscriber must not break others
                logger.exception("progress callback raised for command %s", command_id)

    # -- internals -----------------------------------------------------------------

    async def _execute(self, command: SemanticCommand) -> ToolResult:
        start = time.monotonic()

        def duration() -> float:
            return time.monotonic() - start

        try:
            if command.command_class is CommandClass.READ:
                return await self._execute_read(command, duration)
            return await self._execute_motion(command, duration)
        except Exception as exc:  # noqa: BLE001 - outermost tool-dispatch boundary
            logger.exception("unhandled exception executing command %s", command.command_id)
            command.execution_state = ExecutionState.FAILED
            return self._error(command, duration(), ErrorCode.INTERNAL_ERROR, f"internal error: {exc}")

    async def _execute_read(
        self, command: SemanticCommand, duration: Callable[[], float]
    ) -> ToolResult:
        command.execution_state = ExecutionState.EXECUTING
        handler = self._read_handlers.get(command.operation)
        if handler is None:
            command.execution_state = ExecutionState.FAILED
            return self._error(
                command, duration(), ErrorCode.INTERNAL_ERROR, f"no handler registered for {command.operation.value}"
            )
        result = await handler(command)
        command.execution_state = (
            ExecutionState.SUCCEEDED if result.status == "succeeded" else ExecutionState.FAILED
        )
        return result

    async def _execute_motion(
        self, command: SemanticCommand, duration: Callable[[], float]
    ) -> ToolResult:
        command.execution_state = ExecutionState.SAFETY_CHECK
        current_pose = await self._current_pose_provider()
        outcome = await self._safety_policy_engine.check(command, current_pose)

        if outcome.decision == SafetyDecision.DENY:
            command.execution_state = ExecutionState.SAFETY_REJECTED
            assert outcome.error is not None
            return build_error_result(
                command.expected_result_type,
                command_id=command.command_id,
                robot_id=command.robot_id,
                duration_sec=duration(),
                error=outcome.error,
            )
        if outcome.decision == SafetyDecision.REQUIRE_APPROVAL:
            command.execution_state = ExecutionState.AWAITING_APPROVAL
            return self._awaiting_approval(command, duration(), outcome.approval_reason or "")

        command.execution_state = ExecutionState.PLANNING

        if command.operation is Operation.STOP:
            stop_result: ToolResult = await self._handle_stop(command, duration)
            command.execution_state = ExecutionState.SUCCEEDED
            return stop_result

        # select_backend's return value is only used to confirm availability (None ->
        # CAPABILITY_UNAVAILABLE); dispatch itself goes through the manager's own
        # concretely-typed backend references (self._motion_backend/_navigation_backend,
        # both `Any` at this seam) rather than the Backend = MotionBackend |
        # NavigationBackend union mypy cannot narrow from `command.operation`.
        backend = self._command_planner.select_backend(command.operation, self._registry)
        if backend is None:
            command.execution_state = ExecutionState.CAPABILITY_UNAVAILABLE
            return self._error(
                command,
                duration(),
                ErrorCode.CAPABILITY_UNAVAILABLE,
                f"no backend is currently available for {command.operation.value}",
            )

        token = SimpleCancellationToken(deadline_monotonic=time.monotonic() + command.timeout_s)
        self._tokens[command.command_id] = token
        self._active_commands[command.robot_id] = command
        command.execution_state = ExecutionState.EXECUTING

        result: ToolResult
        try:
            backend_timeout = command.timeout_s + _TIMEOUT_BACKSTOP_MARGIN_S
            if command.operation is Operation.MOVE:
                result = await asyncio.wait_for(
                    self._motion_backend.move(command, token), timeout=backend_timeout
                )
            elif command.operation is Operation.NAVIGATE:
                result = await asyncio.wait_for(
                    self._navigation_backend.navigate(command, token), timeout=backend_timeout
                )
            else:
                result = self._error(
                    command, duration(), ErrorCode.INTERNAL_ERROR,
                    f"unsupported motion operation {command.operation.value}",
                )
        except asyncio.TimeoutError:
            result = self._error(
                command, duration(), ErrorCode.ACTION_TIMEOUT,
                "command exceeded its timeout (execution-manager backstop)",
            )
        finally:
            self._tokens.pop(command.command_id, None)
            if self._active_commands.get(command.robot_id) is command:
                self._active_commands.pop(command.robot_id, None)

        command.execution_state = (
            ExecutionState.SUCCEEDED if result.status == "succeeded" else ExecutionState.FAILED
        )
        return result

    async def _handle_stop(
        self, command: SemanticCommand, duration: Callable[[], float]
    ) -> StopResult:
        cancelled = self._active_commands.get(command.robot_id)
        cancelled_id: str | None = None
        if cancelled is not None and cancelled.command_id != command.command_id:
            cancelled_id = cancelled.command_id
            token = self._tokens.get(cancelled_id)
            if token is not None:
                token.cancel()

        final_pose = None
        if self._motion_backend is not None:
            motion_stop_result = await self._motion_backend.stop()
            final_pose = motion_stop_result.final_pose
        if self._navigation_backend is not None:
            await self._navigation_backend.cancel_all()

        return StopResult(
            status="succeeded",
            command_id=command.command_id,
            robot_id=command.robot_id,
            duration_sec=duration(),
            cancelled_command_id=cancelled_id,
            final_pose=final_pose,
        )

    def _error(
        self, command: SemanticCommand, duration_sec: float, code: ErrorCode, message: str
    ) -> ToolResult:
        return build_error_result(
            command.expected_result_type,
            command_id=command.command_id,
            robot_id=command.robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=code, message=message),
        )

    def _awaiting_approval(
        self, command: SemanticCommand, duration_sec: float, reason: str
    ) -> ToolResult:
        # ToolResult has no dedicated "reason" field for a pending-approval state; the
        # human-readable reason (docs/10-safety-and-trust.md HITL example text) is
        # carried in error.message, the only free-text field the frozen envelope
        # offers, with the closest-fit existing ErrorCode (approval not yet granted).
        return build_result_with_defaults(
            command.expected_result_type,
            status="awaiting_approval",
            command_id=command.command_id,
            robot_id=command.robot_id,
            duration_sec=duration_sec,
            error=ToolError(code=ErrorCode.PERMISSION_DENIED, message=reason, details={"awaiting_approval": True}),
        )
