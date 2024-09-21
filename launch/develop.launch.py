import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator


def generate_launch_description():
    ld = LaunchDescription()

    pkg_rmul24_gazebo_simulator = get_package_share_directory("rmul24_gazebo_simulator")
    ign_config_path = os.path.join(
        pkg_rmul24_gazebo_simulator, "resource", "ign", "gui.config"
    )
    world_sdf_path = os.path.join(
        pkg_rmul24_gazebo_simulator, "resource", "worlds", "rmul_2024_world.sdf"
    )
    robot_xmacro_path = os.path.join(
        pkg_rmul24_gazebo_simulator,
        "resource",
        "xmacro",
        "rmul24_sentry_robot.sdf.xmacro",
    )
    robot_config = os.path.join(
        pkg_rmul24_gazebo_simulator, "config", "base_params.yaml"
    )
    bridge_config = os.path.join(
        pkg_rmul24_gazebo_simulator, "config", "ros_gz_bridge.yaml"
    )
    robot_sdf_path = os.path.join(
        pkg_rmul24_gazebo_simulator, "resource", "xmacro", "rmul24_sentry_robot.sdf"
    )
    referee_config = os.path.join(
        pkg_rmul24_gazebo_simulator, "config", "referee_system_1v1.yaml"
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robots = [
        {
            "name": "red_standard_robot1",
            "color": "red",
            "x_pose": "4.3",
            "y_pose": "3.35",
            "z_pose": "1.2",
            "yaw": "0.0",
        },
        {
            "name": "blue_standard_robot1",
            "color": "blue",
            "x_pose": "10.0",
            "y_pose": "4.5",
            "z_pose": "1.2",
            "yaw": "3.14",
        },
        # ...
    ]

    robot_macro = XMLMacro4sdf()
    robot_macro.set_xml_file(robot_xmacro_path)

    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_file(robot_sdf_path)
    robot_urdf_xml = urdf_generator.to_string()

    declare_position_x_cmd = DeclareLaunchArgument(
        "position_x", default_value="4.3", description="X position of the robot"
    )

    declare_position_y_cmd = DeclareLaunchArgument(
        "position_y", default_value="3.35", description="Y position of the robot"
    )

    declare_orientation_yaw_cmd = DeclareLaunchArgument(
        "orientation_yaw",
        default_value="0.0",
        description="Yaw orientation of the robot",
    )

    # Set Gazebo plugin path
    append_enviroment_worlds = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(pkg_rmul24_gazebo_simulator, "resource", "worlds"),
    )

    append_enviroment_models = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.join(pkg_rmul24_gazebo_simulator, "resource", "models"),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "ign_args": world_sdf_path + " --gui-config " + ign_config_path,
        }.items(),
    )

    # robot base for each robot
    for robot in robots:

        robot_macro.generate({"global_initial_color": robot["color"]})
        robot_xml = robot_macro.to_string()
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )
        ld.add_action(spawn_robot)

        # replace the robot name in the bridge config file
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[
                robot_config,
                {"robot_name": robot["name"]},
            ],
            output="screen",
        )
        ld.add_action(robot_base)

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            parameters=[
                {
                    "robot_description": robot_urdf_xml,
                    "publish_frequency": 20.0,
                }
            ],
            remappings=remappings,
        )
        ld.add_action(robot_state_publisher)

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )
        ld.add_action(robot_ign_bridge)

        # Execute service call after spawning robots
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )
        ld.add_action(set_performer_service)

    # referee system
    referee_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="referee_system",
        arguments=[
            "/referee_system/attack_info@std_msgs/msg/String[ignition.msgs.StringMsg",
            "/referee_system/shoot_info@std_msgs/msg/String[ignition.msgs.StringMsg",
        ],
        remappings=[
            ("/referee_system/attack_info", "/referee_system/ign/attack_info"),
            ("/referee_system/shoot_info", "/referee_system/ign/shoot_info"),
        ],
    )
    referee_ign_bridge2 = Node(
        package="rmoss_gz_bridge",
        executable="pose_bridge",
        namespace="referee_system",
        parameters=[{"robot_filter": True}],
    )
    referee_ign_rfid_bridge2 = Node(
        package="rmoss_gz_bridge",
        executable="rfid_bridge",
        namespace="referee_system",
        parameters=[],
    )
    referee_system = Node(
        package="rmul24_gazebo_simulator",
        executable="simple_competition_1v1.py",
        namespace="referee_system",
        parameters=[referee_config],
    )

    ld.add_action(append_enviroment_worlds)
    ld.add_action(append_enviroment_models)
    ld.add_action(declare_position_x_cmd)
    ld.add_action(declare_position_y_cmd)
    ld.add_action(declare_orientation_yaw_cmd)
    ld.add_action(gazebo)
    ld.add_action(referee_system)
    ld.add_action(referee_ign_bridge2)
    ld.add_action(referee_ign_rfid_bridge2)
    ld.add_action(referee_ign_bridge)

    return ld
