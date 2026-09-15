#!/usr/bin/env bash
# Brings up turtlebot3_gazebo (gzserver only — no gzclient GUI, no X server in this
# container — via headless_world.launch.py: same composition as the package's own
# turtlebot3_world.launch.py minus the gzclient include), robot_state_publisher, robot
# spawn. Nav2+AMCL are NOT started here — the acceptance run needs a without-Nav2 phase
# (criterion 2) before a with-Nav2 phase, so Nav2 is launched separately (see
# acceptance/RESULTS.md for the exact command used) once this container is already up.
set -eo pipefail
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

exec ros2 launch /headless_world.launch.py use_sim_time:=true
