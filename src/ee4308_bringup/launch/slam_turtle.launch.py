from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ee4308_bringup = FindPackageShare("ee4308_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
    # use_sim_time
    arg_configuration_directory = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="'True' to launch in simulation. 'False' for real-time.",
    )
    ld.add_action(arg_configuration_directory)

    # Configuration file's parent directory.
    arg_configuration_directory = DeclareLaunchArgument(
        "configuration_directory",
        default_value=PathJoinSubstitution([pkg_ee4308_bringup, "params"]),
        description="The directory containing the lua config file to load",
    )
    ld.add_action(arg_configuration_directory)

    # Configuration file's name.
    arg_configuration_basename = DeclareLaunchArgument(
        "configuration_basename",
        default_value="slam_turtle.lua",
        description="Name of lua config file for cartographer.",
    )
    ld.add_action(arg_configuration_basename)

    # Resolution
    arg_resolution = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Resolution of a grid cell in the published occupancy grid",
    )
    ld.add_action(arg_resolution)

    # Period sec for occupancy grid
    arg_publish_period_sec = DeclareLaunchArgument(
        "publish_period_sec",
        default_value="1.0",
        description="OccupancyGrid publishing period",
    )
    ld.add_action(arg_publish_period_sec)

    # node
    node_cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory",
            LaunchConfiguration("configuration_directory"),
            "-configuration_basename",
            LaunchConfiguration("configuration_basename"),
        ],
    )
    ld.add_action(node_cartographer)

    node_cartographer_occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-resolution",
            LaunchConfiguration("resolution"),
            "-publish_period_sec",
            LaunchConfiguration("publish_period_sec"),
        ],
    )
    ld.add_action(node_cartographer_occupancy_grid)

    return ld
