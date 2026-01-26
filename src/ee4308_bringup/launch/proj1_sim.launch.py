from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    # Launch Arg: turtle_x
    arg_turtle_x = DeclareLaunchArgument(
        "turtle_x", default_value="-2.0", description="x position of the turtle."
    )
    ld.add_action(arg_turtle_x)

    # Launch Arg: turtle_y
    arg_turtle_y = DeclareLaunchArgument(
        "turtle_y", default_value="-0.5", description="y position of the turtle."
    )
    ld.add_action(arg_turtle_y)

    arg_map = DeclareLaunchArgument(
        "map",
        default_value="proj1_sim",
        description="Name of the yaml map file to load. Must be in ee4308_bringup/maps, and without the file extension.",
    )
    ld.add_action(arg_map)

    arg_nav2_param_file = DeclareLaunchArgument(
        "nav2_param_file",
        default_value="proj1",
        description="Name of the yaml map file to load the nav2 parameters. Must be in ee4308_bringup/params, and without the file extension.",
    )
    ld.add_action(arg_nav2_param_file)

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

    # nav2 turtle
    launch_nav2_turtle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee4308_bringup, "launch", "nav2_turtle.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "map": LaunchConfiguration("map"),
            "nav2_param_file": LaunchConfiguration("nav2_param_file"),
        }.items(),
    )
    ld.add_action(launch_nav2_turtle)

    
    return ld
