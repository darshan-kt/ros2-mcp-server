#!/usr/bin/env bash
# Brings up turtlebot3_gazebo (gzserver + gzclient under xvfb, robot_state_publisher,
# robot spawn). Nav2+AMCL are NOT started here — the acceptance run needs a
# without-Nav2 phase (criterion 2) before a with-Nav2 phase, so Nav2 is launched
# separately (see acceptance/RESULTS.md for the exact command used) once this
# container is already up.
set -eo pipefail
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

exec xvfb-run -a -s "-screen 0 1024x768x24" \
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=true
