# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_map_path=os.path.join(
        os.path.expanduser('~'), 'maps', 'iscas_museum', 'iscas_museum_2d.yaml'
    )
    map_path=LaunchConfiguration('map_path')
    declare_map_path=DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
    )

    param_file_name=LaunchConfiguration('param_file_name')
    declare_param_file_name=DeclareLaunchArgument(
        'param_file_name',
        default_value='iscas_museum.param.yaml',
    )
    param_path=PathJoinSubstitution([
        FindPackageShare('mugimaru_navigation2'), 'config', param_file_name
    ])

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )
    autostart=LaunchConfiguration('autostart')
    declare_autostart=DeclareLaunchArgument(
        'autostart',
        default_value='True',
    )

    lifecycle_nodes=[
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    param_list=[
        param_path,
        {'use_sim_time': use_sim_time},
    ]

    map_server=Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=param_list + [{'yaml_filename': map_path}],
        output='screen',
    )
    controller_server=Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=param_list,
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        output='screen',
    )
    smoother_server=Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=param_list,
        output='screen',
    )
    planner_server=Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=param_list,
        output='screen',
    )
    behavior_server=Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=param_list,
        output='screen',
    )
    bt_navigator=Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=param_list,
        output='screen',
    )
    waypoint_follower=Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=param_list,
        output='screen',
    )
    velocity_smoother=Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=param_list,
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
        output='screen',
    )
    lifecycle_manager_navigation=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=param_list + [
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_map_path)
    ld.add_action(declare_param_file_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)

    ld.add_action(map_server)
    ld.add_action(controller_server)
    ld.add_action(smoother_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(lifecycle_manager_navigation)

    return ld
