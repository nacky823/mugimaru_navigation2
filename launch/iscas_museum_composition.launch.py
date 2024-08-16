# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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

    map_server=ComposableNode(
        package='nav2_map_server',
        plugin='nav2_map_server::MapServer',
        name='map_server',
        parameters=[
            param_path,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_path,
            },
        ],
    )
    controller_server=ComposableNode(
        package='nav2_controller',
        plugin='nav2_controller::ControllerServer',
        name='controller_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    smoother_server=ComposableNode(
        package='nav2_smoother',
        plugin='nav2_smoother::SmootherServer',
        name='smoother_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
    )
    planner_server=ComposableNode(
        package='nav2_planner',
        plugin='nav2_planner::PlannerServer',
        name='planner_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
    )
    behavior_server=ComposableNode(
        package='nav2_behaviors',
        plugin='behavior_server::BehaviorServer',
        name='behavior_server',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
    )
    bt_navigator=ComposableNode(
        package='nav2_bt_navigator',
        plugin='nav2_bt_navigator::BtNavigator',
        name='bt_navigator',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
    )
    waypoint_follower=ComposableNode(
        package='nav2_waypoint_follower',
        plugin='nav2_waypoint_follower::WaypointFollower',
        name='waypoint_follower',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
    )
    velocity_smoother=ComposableNode(
        package='nav2_velocity_smoother',
        plugin='nav2_velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )
    lifecycle_manager_navigation=ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation',
        parameters=[
            param_path,
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            },
        ],
    )

    container=ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            map_server,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager_navigation,
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(container)

    return ld
