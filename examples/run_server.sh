#!/usr/bin/env bash
# Launches ROS-MCP-Server with the ROS 2 environment sourced. Claude Desktop starts its
# configured MCP server command in a plain (non-login) shell that does not have ROS
# sourced, so this wrapper is what claude_desktop_config.json actually points at.
set -eo pipefail
# Not `set -u`: ROS 2's setup.bash itself references variables that are unset on a
# clean shell (e.g. AMENT_TRACE_SETUP_FILES) and expects normal bash semantics there.

ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

exec python3 -m ros_mcp.server --config "${REPO_ROOT}/config/robot.yaml" "$@"
