import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    follower_dir = get_package_share_directory("tb3_follower")
    slamtoolbox_launch_dir = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch"
    )
    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_config_file_path = (
        "/opt/ros/galactic/share/nav2_bringup/rviz/nav2_default_view.rviz"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(follower_dir, "config", "nav2_follower_params.yaml"),
        description=(
            "Full path to the ROS2 parameters file to use for all launched nodes"
        ),
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=rviz_config_file_path,
        description="Full path to the RVIZ config file to use",
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slamtoolbox_launch_dir, "localization_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "use_lifecycle_mgr": "false",
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
        ]
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    update_pose_cmd = Node(
        package="tb3_follower",
        executable="clicked_point_to_pose",
        name="clicked_point_to_pose",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(update_pose_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
