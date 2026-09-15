# ROS-MCP-Server

A stable semantic robotics API, exposed over MCP, backed by an unmodified ROS 2 graph.
Claude reasons in terms of `robot.move`, `robot.navigate`, `robot.get_state`, …; the
server discovers what a given robot actually supports and translates those calls into
the right ROS 2 mechanism — `/cmd_vel`, Nav2, a sensor topic — with a deterministic
safety layer the LLM cannot bypass.

The full architecture (design principles, frozen contracts, ADRs, safety model) lives
in [`docs/`](docs/00-overview.md). This file covers running the MVP.

## Requirements

- ROS 2 Humble (sourced), Python 3.10
- `turtlebot3_gazebo` (or another robot with the topics/action named in
  [`config/robot.yaml`](config/robot.yaml))
- Python packages: `mcp`, `pydantic`, `numpy`, `pyyaml`, `Pillow` (see
  [`pyproject.toml`](pyproject.toml))

## Setup

```bash
source /opt/ros/humble/setup.bash
cd ros2-mcp-server
python3 -m pip install --user mcp pydantic numpy pyyaml Pillow
python3 -m pytest tests/ -q                       # should show "N passed", ROS mocked
python3 -m ros_mcp.server --config config/robot.yaml   # starts even with no robot present
```

If `python3 -m ros_mcp.server` reports `No module named 'ros_mcp'`, either
`pip install -e .` (needs `setuptools>=64` for a PEP 660 editable install) or add
`src/` to `PYTHONPATH` — `examples/run_server.sh` does the latter automatically and is
what the Claude Desktop config below actually launches.

## Connect Claude Desktop

1. Copy the `mcpServers` entry from [`examples/claude_desktop_config.json`](examples/claude_desktop_config.json).
2. Point its `command` at this repo's `examples/run_server.sh` (absolute path).
3. Set `ROS_DISTRO_SETUP` if your ROS setup script isn't `/opt/ros/humble/setup.bash`.
4. Restart Claude Desktop.
5. Ask it to move the robot, navigate somewhere, or describe what it sees.

`run_server.sh` sources ROS and execs the server — Desktop launches MCP servers with no
ROS environment of their own, so a bare `python -m ros_mcp.server` command won't work.

## Reference Robot

[`config/robot.yaml`](config/robot.yaml) targets TurtleBot3 (Waffle) under
`turtlebot3_gazebo`: `/cmd_vel`, `/odom`, `/scan`, `/camera/image_raw`, `/tf`, and,
when Nav2 + AMCL are also running, `/navigate_to_pose` + `/amcl_pose`. Discovery infers
capabilities from whatever is actually present — the tool list degrades cleanly
(`robot.navigate` simply doesn't appear, or returns `CAPABILITY_UNAVAILABLE`) when Nav2
or localization isn't running, and `robot.move`/`robot.stop`/the read tools work with
just the sim's base topics.

TurtleBot3's TF tree uses `base_footprint` as the moving base frame (`base_link` is a
static child of it), not the `base_link` name the docs use generically — set
`ROS_MCP_BASE_FRAME=base_footprint` (already in `run_server.sh`'s env for this repo) if
you run against a different platform that does use `base_link` directly, leave it unset.

## Tests

```bash
python3 -m pytest tests/ -q                                              # unit suite, ROS mocked
MYPYPATH=src python3 -m mypy -p ros_mcp                                  # type check
MYPYPATH=src:tests python3 -m mypy -p tests.typecheck.protocol_conformance  # Protocol conformance
```

The unit suite needs no live ROS graph. `tests/typecheck/protocol_conformance.py` is
not collected by pytest; it's a static check (each concrete class assigned to a
variable typed as its frozen `docs/13-contracts.md` Protocol, verified by mypy since
only `CancellationToken` is `@runtime_checkable`).

Live-robot testing (`get_state`, `get_laser_scan`, `move`, `stop`, `navigate`) requires
`turtlebot3_gazebo` running and, for `navigate`, Nav2 + AMCL with an initial pose set
(`ros2 topic pub /initialpose ...` once, or set it in RViz).
