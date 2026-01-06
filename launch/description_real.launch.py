import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")

    pkg_share = FindPackageShare("go2_description").find("go2_description")
    default_model_path = os.path.join(pkg_share, "xacro", "robot.xacro")
    rviz_config_path = os.path.join(pkg_share, "rviz", "go2.rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )
    declare_description_path = DeclareLaunchArgument(
        "description_path", default_value=default_model_path
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command([
                    os.path.expanduser("~/.local/bin/xacro"),
                    " ",
                    description_path
                ])
            },
            {"use_sim_time": use_sim_time},
        ],
    )
    
    topic_node = Node(
        package="go2_state_bridge",
        executable="lowstate_to_jointstate",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],  
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_description_path,
        topic_node,
        robot_state_publisher_node,
        rviz_node,
    ])
