from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    # Launch Arg: x
    arg_turtle_x = DeclareLaunchArgument(
        "turtle_x", default_value="-2.0", description="x position of the turtle."
    )
    ld.add_action(arg_turtle_x)

    # Launch Arg: turtle_y
    arg_turtle_y = DeclareLaunchArgument(
        "turtle_y", default_value="-0.5", description="y position of the turtle."
    )
    ld.add_action(arg_turtle_y)

    # Launch Arg: world
    arg_world = DeclareLaunchArgument(
        "world",
        default_value="turtlebot3_house",
        description="Name of the Gazebo world file to load (without .world extension). Must be in ee4308_bringup/worlds",
    )  # Worlds must have a more relaxed contact coefficient (~100) for faster solving.
    ld.add_action(arg_world)

    # Launch Arg: headless
    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="If set to True, open the Gazebo simulator and its GUI. If set to False, the GUI is gone and the simulation runs in the background.",
    )
    ld.add_action(arg_headless)

    # Launch Arg: libgl
    arg_libgl = DeclareLaunchArgument(
        "libgl",
        default_value="False",
        description="If set to True, LibGL is used (this is primarily for VirtualBox users). If set to False, the faster ogre2 renderer is used (cannot be used in VirtualBox, only for Dual boot).",
    )
    ld.add_action(arg_libgl)

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

    # sim
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "sim.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "libgl": LaunchConfiguration("libgl"),
        }.items(),
    )
    ld.add_action(launch_sim)

    # spawn models
    launch_spawn_models = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "spawn_models.launch.py"])
        ),
        launch_arguments={
            "project": "1",
            "turtle_x": LaunchConfiguration("turtle_x"),
            "turtle_y": LaunchConfiguration("turtle_y"),
        }.items(),
    )
    ld.add_action(launch_spawn_models)

    # slam turtle
    launch_slam_turtle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "slam_turtle.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "configuration_directory": LaunchConfiguration("configuration_directory"),
            "configuration_basename": LaunchConfiguration("configuration_basename"),
            "resolution": LaunchConfiguration("resolution"),
            "publish_period_sec": LaunchConfiguration("publish_period_sec"),
        }.items(),
    )
    ld.add_action(launch_slam_turtle)

    # rviz
    path_rviz = PathJoinSubstitution([pkg_ee4308_bringup, "rviz", "proj1_slam.rviz"])
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", path_rviz],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    ld.add_action(node_rviz)

    return ld
