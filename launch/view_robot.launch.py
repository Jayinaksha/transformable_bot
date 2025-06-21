#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # ===== Set Gazebo plugin path to find our custom plugin =====
    package_dir = get_package_share_directory('transformable_bot')
    plugin_path = os.path.join(
        os.path.dirname(os.path.dirname(package_dir)),
        'lib', 'transformable_bot'
    )
    if 'GZ_SIM_SYSTEM_PLUGIN_PATH' in os.environ:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugin_path + ':' + os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']
    else:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugin_path

    print(f"Setting GZ_SIM_SYSTEM_PLUGIN_PATH to: {os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']}")

    # ===== Robot description (XACRO → URDF) =====
    xacro_file = PathJoinSubstitution([
        FindPackageShare('transformable_bot'),
        'urdf',
        'transformable_bot.urdf.xacro'
    ])
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # ===== Joint State Publisher GUI =====
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )

    # ===== RViz2 =====
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=[
    #         '-d', PathJoinSubstitution([
    #             FindPackageShare('transformable_bot'),
    #             'config',
    #             'view_robot.rviz'
    #         ])
    #     ]
    # )

    # ===== Launch Gazebo =====
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', 'empty.sdf'],
        output='screen'
    )
    
    # ===== Spawn robot in Gazebo =====
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'transformable_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Optionally, you could add DeclareLaunchArgument for enabling/disabling these:
        # DeclareLaunchArgument('gui', default_value='true', description='Enable joint_state_publisher_gui'),
        # DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz'),

        robot_state_publisher,
        # joint_state_publisher_gui,
        # rviz,
        gazebo,
        spawn_robot
    ])