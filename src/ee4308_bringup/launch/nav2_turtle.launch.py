import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
    Command,
)


def generate_launch_description():
    ld = LaunchDescription()

    pkg_nav2_bringup = FindPackageShare("nav2_bringup")
    pkg_ee4308_bringup = FindPackageShare("ee4308_bringup")

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="'True' for simulation, 'False' otherwise.",
    )
    ld.add_action(arg_use_sim_time)

    arg_map = DeclareLaunchArgument(
        "map",
        default_value="lab",
        description="Name of the yaml map file to load. Must be in ee4308_bringup/maps, and without the file extension.",
    )
    ld.add_action(arg_map)

    arg_nav2_param_file = DeclareLaunchArgument(
        "nav2_param_file",
        default_value="proj1",
        description="Name of the yaml map file to load the nav2 parameters. Must be in ee4308_bringup/params, and without the file extension.",
    )
    ld.add_action(arg_nav2_param_file)

    path_map = PathJoinSubstitution(
        [pkg_ee4308_bringup, "maps", [LaunchConfiguration("map"), ".yaml"]]
    )
    path_nav2_params = PathJoinSubstitution(
        [pkg_ee4308_bringup, "params", [LaunchConfiguration("nav2_param_file"), ".yaml"]]
    )

    # nav2 bring_up
    launch_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "map": path_map,
            "params_file": path_nav2_params,
            # "namespace": "turtle", # nav2 has poor support for namespaces
        }.items(),
    )
    ld.add_action(launch_nav2_bringup)

    # nav2 rviz2
    launch_nav2_rviz= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, "launch", "rviz_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            # "namespace": "turtle", # nav2 has poor support for namespaces
        }.items(),
    )
    ld.add_action(launch_nav2_rviz)

    return ld
