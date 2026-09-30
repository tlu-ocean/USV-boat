#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def pv(name, value_type):
    return ParameterValue(
        LaunchConfiguration(name),
        value_type=value_type,
    )


def generate_launch_description():

    args = [

        DeclareLaunchArgument(
            'lidar_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
                'Controller_c08b3aa4146ff0118af8539b1045c30f-if00-port0'
            ),
        ),

        DeclareLaunchArgument(
            'front_angle_deg',
            default_value='0.0',
        ),

        # IMPORTANT:
        # Replace these with measured boat geometry.
        DeclareLaunchArgument(
            'corridor_start_x_m',
            default_value='0.40',
        ),

        DeclareLaunchArgument(
            'corridor_half_width_m',
            default_value='0.45',
        ),

        DeclareLaunchArgument(
            'corridor_lookahead_m',
            default_value='5.0',
        ),

        # First-water-test conservative starting values.
        DeclareLaunchArgument(
            'blocked_enter_m',
            default_value='1.5',
        ),

        DeclareLaunchArgument(
            'blocked_exit_m',
            default_value='1.8',
        ),

        DeclareLaunchArgument(
            'caution_enter_m',
            default_value='3.0',
        ),

        DeclareLaunchArgument(
            'caution_exit_m',
            default_value='3.5',
        ),

        DeclareLaunchArgument(
            'planner_min_safe_clearance_m',
            default_value='1.5',
        ),

        DeclareLaunchArgument(
            'planner_target_distance_m',
            default_value='2.0',
        ),
    ]

    nodes = [

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 1000000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
            }],
            output='log',
        ),

        Node(
            package='boat_perception',
            executable='lidar_sector_node',
            name='lidar_sector_node',
            parameters=[{
                'front_angle_deg': pv(
                    'front_angle_deg',
                    float,
                ),
                'min_obstacle_range_m': 0.05,
                'corridor_start_x_m': pv(
                    'corridor_start_x_m',
                    float,
                ),
                'corridor_half_width_m': pv(
                    'corridor_half_width_m',
                    float,
                ),
                'corridor_lookahead_m': pv(
                    'corridor_lookahead_m',
                    float,
                ),
            }],
            output='log',
        ),

        Node(
            package='boat_perception',
            executable='lidar_obstacle_node',
            name='lidar_obstacle_node',
            parameters=[{
                'blocked_enter_m': pv(
                    'blocked_enter_m',
                    float,
                ),
                'blocked_exit_m': pv(
                    'blocked_exit_m',
                    float,
                ),
                'caution_enter_m': pv(
                    'caution_enter_m',
                    float,
                ),
                'caution_exit_m': pv(
                    'caution_exit_m',
                    float,
                ),
                'release_confirm_cycles': 3,
                'stale_timeout_s': 0.5,
                'side_margin_m': 0.10,
            }],
            output='log',
        ),

        Node(
            package='boat_navigation',
            executable='avoidance_decision_node',
            name='avoidance_decision_node',
            parameters=[{
                'stale_timeout_s': 0.75,
                'turn_acquire_clearance_m': pv(
                    'blocked_enter_m',
                    float,
                ),
                'turn_hold_clearance_m': 1.0,
            }],
            output='screen',
        ),

        Node(
            package='boat_navigation',
            executable='local_planner_node',
            name='local_planner_node',
            parameters=[{
                'front_angle_deg': pv(
                    'front_angle_deg',
                    float,
                ),
                'corridor_start_x_m': pv(
                    'corridor_start_x_m',
                    float,
                ),
                'corridor_half_width_m': pv(
                    'corridor_half_width_m',
                    float,
                ),
                'lookahead_m': pv(
                    'corridor_lookahead_m',
                    float,
                ),
                'min_safe_clearance_m': pv(
                    'planner_min_safe_clearance_m',
                    float,
                ),
                'nominal_target_distance_m': pv(
                    'planner_target_distance_m',
                    float,
                ),
                'candidate_step_deg': 15.0,
                'max_turn_deg': 75.0,
                'min_cluster_points': 8,
                'max_index_gap': 3,
                'max_range_jump_m': 0.30,
                'turn_penalty_m_per_deg': 0.01,
            }],
            output='screen',
        ),

        Node(
            package='boat_navigation',
            executable='guided_target_node',
            name='guided_target_node',
            parameters=[{
                'stale_timeout_s': 0.75,
                'planner_timeout_s': 0.75,
            }],
            output='screen',
        ),

        Node(
            package='boat_sensors',
            executable='pixhawk_node',
            name='pixhawk_node',
            parameters=[{
                'port': '/dev/serial0',
                'baud': 115200,

                # Control infrastructure is enabled,
                # but both safety master switches start OFF.
                'enable_guided_control': True,
                'enable_auto_restore': True,
                'enable_guided_target_control': True,

                'autonomy_master_enable': False,
                'allow_control_when_armed': False,
            }],
            output='screen',
        ),
    ]

    return LaunchDescription(
        args + nodes
    )
