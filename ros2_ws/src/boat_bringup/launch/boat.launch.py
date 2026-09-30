from datetime import datetime
import os

import yaml

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)

from launch.conditions import (
    IfCondition,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)


def load_record_topics(filename):

    with open(filename, "r") as file:
        config = yaml.safe_load(file)

    return config.get("topics", [])


def generate_launch_description():

    # ========================================================
    # Package paths
    # ========================================================

    package_share = (
        get_package_share_directory(
            "boat_bringup"
        )
    )

    config_file = os.path.join(
        package_share,
        "config",
        "boat.yaml",
    )

    record_topics_file = os.path.join(
        package_share,
        "config",
        "record_topics.yaml",
    )

    record_topics = load_record_topics(
        record_topics_file
    )

    # ========================================================
    # Launch arguments
    # ========================================================

    enable_pixhawk = LaunchConfiguration(
        "enable_pixhawk"
    )

    enable_tfmini = LaunchConfiguration(
        "enable_tfmini"
    )

    enable_bme280 = LaunchConfiguration(
        "enable_bme280"
    )

    enable_ais = LaunchConfiguration(
        "enable_ais"
    )

    record = LaunchConfiguration(
        "record"
    )

    bag_dir = LaunchConfiguration(
        "bag_dir"
    )

    bag_prefix = LaunchConfiguration(
        "bag_prefix"
    )

    # Timestamp is created once when launch starts.
    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    bag_name = [
        bag_prefix,
        "_",
        timestamp,
    ]

    bag_output = PathJoinSubstitution([
        bag_dir,
        bag_name,
    ])

    # ========================================================
    # Description
    # ========================================================

    return LaunchDescription([

        # ----------------------------------------------------
        # Hardware enable / disable
        # ----------------------------------------------------

        DeclareLaunchArgument(
            "enable_pixhawk",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "enable_tfmini",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "enable_bme280",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "enable_ais",
            default_value="true",
        ),

        # ----------------------------------------------------
        # Recording
        # ----------------------------------------------------

        DeclareLaunchArgument(
            "record",
            default_value="false",
        ),

        DeclareLaunchArgument(
            "bag_dir",
            default_value=(
                os.path.expanduser(
                    "~/boat/data/rosbags"
                )
            ),
        ),

        DeclareLaunchArgument(
            "bag_prefix",
            default_value="boat",
        ),

        # ====================================================
        # Pixhawk
        # ====================================================

        Node(
            package="boat_sensors",
            executable="pixhawk_node",
            name="pixhawk_node",

            parameters=[
                config_file,
            ],

            condition=IfCondition(
                enable_pixhawk
            ),

            output="screen",
        ),

        # ====================================================
        # TFmini
        # ====================================================

        Node(
            package="boat_sensors",
            executable="tfmini_node",
            name="tfmini_node",

            parameters=[
                config_file,
            ],

            condition=IfCondition(
                enable_tfmini
            ),

            output="screen",
        ),

        # ====================================================
        # BME280
        # ====================================================

        Node(
            package="boat_sensors",
            executable="bme280_node",
            name="bme280_node",

            parameters=[
                config_file,
            ],

            condition=IfCondition(
                enable_bme280
            ),

            output="screen",
        ),

        # ====================================================
        # AIS
        # ====================================================

        Node(
            package="boat_sensors",
            executable="ais_node",
            name="ais_node",

            parameters=[
                config_file,
            ],

            condition=IfCondition(
                enable_ais
            ),

            output="screen",
        ),

        # ====================================================
        # ROS bag recorder
        # ====================================================

        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",

                "-o",
                bag_output,

                "--topics",

                *record_topics,
            ],

            condition=IfCondition(
                record
            ),

            output="screen",
        ),
    ])
