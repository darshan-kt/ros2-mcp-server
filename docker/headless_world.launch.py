"""Same composition as turtlebot3_gazebo's turtlebot3_world.launch.py (gzserver,
robot_state_publisher, spawn_turtlebot3) but WITHOUT gzclient — this container has no
X server, and getting xvfb-run to hand off to the wrapped command reliably in this base
image was not worth chasing when the acceptance run only needs gzserver's ROS 2
topics/actions, never the GUI."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="-0.5")

    world = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "worlds", "turtlebot3_world.world")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    # spawn_entity.py has a fixed internal 30s wait for the /spawn_entity service and no
    # exposed timeout override; on this shared, heavily-loaded host gzserver has been
    # observed taking longer than that to finish initializing (ALSA/OpenAL probing during
    # startup logs after the spawn attempt already gave up). Delaying the spawn attempt
    # gives gzserver a head start so its own 30s retry window is enough.
    delayed_spawn_cmd = TimerAction(period=25.0, actions=[spawn_turtlebot_cmd])

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(delayed_spawn_cmd)
    return ld
