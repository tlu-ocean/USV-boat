from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


LIDAR_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "c08b3aa4146ff0118af8539b1045c30f-if00-port0"
)



BAG_ROOT = os.path.expanduser(
    "~/boat/data/rosbags"
)

BAG_TOPICS = [
    # FCU / navigation
    "/mavros/state",
    "/mavros/sys_status",

    # GPS / ownship
    "/mavros/global_position/raw/fix",
    "/mavros/global_position/raw/satellites",
    "/mavros/global_position/raw/gps_vel",
    "/mavros/global_position/global",
    "/mavros/global_position/compass_hdg",

    # Navigation controller
    "/mavros/nav_controller_output/output",

    # S2 obstacle perception
    "/mavros/obstacle/send",

    # TFmini
    "/sensors/tfmini/range",
    "/sensors/tfmini/strength",
    "/sensors/tfmini/internal_temperature",

    # AIS
    "/ais/serial_connected",
    "/ais/rx_active",
    "/ais/target_count",
    "/ais/targets",

    # Collision / CPA / TCPA / COLREG
    "/collision/status",
    "/collision/targets",
    "/collision/risk",
    "/collision/cpa_m",
    "/collision/tcpa_s",

    # Runtime messages
    "/rosout",
]


def make_recorder(context):

    record_value = (
        LaunchConfiguration("record")
        .perform(context)
        .strip()
        .lower()
    )

    if record_value not in (
        "1",
        "true",
        "yes",
        "on",
    ):
        return []

    prefix = (
        LaunchConfiguration("bag_prefix")
        .perform(context)
        .strip()
    )

    if not prefix:
        prefix = "boat"

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    os.makedirs(
        BAG_ROOT,
        exist_ok=True,
    )

    output_path = os.path.join(
        BAG_ROOT,
        f"{prefix}_{stamp}",
    )

    print(
        f"[boat_logging] recording to: "
        f"{output_path}"
    )

    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                output_path,
                *BAG_TOPICS,
            ],
            output="screen",
        )
    ]


def generate_launch_description():

    mavros_share = get_package_share_directory("mavros")
    boat_share = get_package_share_directory("boat_bringup")

    apm_config = os.path.join(
        mavros_share,
        "launch",
        "apm_config.yaml",
    )

    boat_pluginlist = os.path.join(
        boat_share,
        "config",
        "mavros_plugins_boat.yaml",
    )

    boat_override = os.path.join(
        boat_share,
        "config",
        "mavros_boat_override.yaml",
    )

    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            {
                "fcu_url": "/dev/serial0:115200",
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
            boat_pluginlist,
            apm_config,
            boat_override,
        ],
    )

    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": LIDAR_PORT,
                "serial_baudrate": 1000000,
                "frame_id": "laser",
                "inverted": True,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
            }
        ],
        remappings=[
            ("scan", "/mavros/obstacle/send"),
        ],
    )

    tfmini = Node(
        package="boat_sensors",
        executable="tfmini_node",
        name="tfmini_node",
        output="screen",
    )

    ais = Node(
        package="boat_sensors",
        executable="ais_node",
        name="ais_node",
        output="screen",
        parameters=[
            {
                "port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
                "baud": 38400,
            }
        ],
    )

    collision = Node(
        package="boat_collision",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            {
                "test_mode": False,
                "ais_topic": "/ais/targets",
                "ownship_min_speed_mps": 0.5,

                # Development advisory thresholds.
                "warn_cpa_m": 50.0,
                "warn_tcpa_s": 300.0,
            }
        ],
    )

    health = Node(
        package="boat_bringup",
        executable="boat_health",
        name="boat_health",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "record",
            default_value="false",
            description="Enable rosbag recording",
        ),

        DeclareLaunchArgument(
            "bag_prefix",
            default_value="water_test",
            description="Prefix for rosbag directory",
        ),

        mavros,
        lidar,
        tfmini,
        ais,
        collision,
        health,
        OpaqueFunction(function=make_recorder),
    ])
