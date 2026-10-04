from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


LIDAR_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "c08b3aa4146ff0118af8539b1045c30f-if00-port0"
)


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

    return LaunchDescription([
        mavros,
        lidar,
    ])
