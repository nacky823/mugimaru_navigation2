# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    map_path=os.path.join(
        os.path.expanduser('~'), 'maps', 'iscas_museum', 'iscas_museum_2d.yaml'
    )
    param_path=os.path.join(
        get_package_share_directory('mugimaru_navigation2'),
        'config', 'iscas_museum.param.yaml'
    )

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

    map_server=Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            param_path,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_path,
            },
        ],
        output='screen',
    )
    controller_server=Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        output='screen',
    )
    smoother_server=Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    planner_server=Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    behavior_server=Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    bt_navigator=Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    waypoint_follower=Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    velocity_smoother=Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
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
        parameters=[
            param_path,
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            },
        ],
        output='screen',
    )

    ld=LaunchDescription()

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
