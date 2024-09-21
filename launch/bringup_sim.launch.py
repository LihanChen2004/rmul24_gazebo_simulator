import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmul24_gazebo_simulator")

    world_sdf_path = os.path.join(pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf")
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_simulator, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")),
        launch_arguments={
            "robots_init_pose_path": os.path.join(pkg_simulator, "config", "robots_init_pose.yaml"),
        }.items(),
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_simulator, "launch", "referee_system.launch.py"))
    )

    ld = LaunchDescription()

    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robots_launch)
    ld.add_action(referee_system_launch)

    return ld
