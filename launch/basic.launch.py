#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    TURTLEBOT3_MODEL = "waffle"
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    sdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        model_folder,
        "model.sdf",
    )
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="-0.5")

    world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "worlds",
        # "empty_world.world",
        "turtlebot3_world.world",
        # "turtlebot3_house.world",
    )

    world = os.path.join(
        get_package_share_directory("articubot_one"),
        "worlds",
        "obstacles.world",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "urdf", urdf_file_name
    )

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", urdf_path]),
            }
        ],
    )

    start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            TURTLEBOT3_MODEL,
            "-file",
            sdf_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.01",
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
