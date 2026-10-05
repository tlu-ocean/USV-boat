#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


EARTH_RADIUS_M = 6371000.0


class AISTestTarget(Node):

    def __init__(self):
        super().__init__('ais_test_target')

        self.declare_parameter('range_m', 100.0)
        self.declare_parameter('bearing_deg', 0.0)
        self.declare_parameter('sog_knots', 0.0)
        self.declare_parameter('cog_deg', 0.0)

        self.fix = None

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.fix_cb,
            qos_profile_sensor_data,
        )

        self.pub = self.create_publisher(
            String,
            '/ais/test_targets',
            10,
        )

        self.create_timer(
            0.5,
            self.publish_target,
        )

        self.get_logger().info(
            'AIS test target publisher started'
        )

    def fix_cb(self, msg):
        self.fix = msg

    def publish_target(self):

        if self.fix is None:
            return

        if self.fix.status.status < 0:
            return

        lat = self.fix.latitude
        lon = self.fix.longitude

        if not (
            math.isfinite(lat)
            and math.isfinite(lon)
        ):
            return

        range_m = self.get_parameter(
            'range_m'
        ).value

        bearing_deg = self.get_parameter(
            'bearing_deg'
        ).value

        sog_knots = self.get_parameter(
            'sog_knots'
        ).value

        cog_deg = self.get_parameter(
            'cog_deg'
        ).value

        bearing = math.radians(
            bearing_deg
        )

        north = (
            range_m
            * math.cos(bearing)
        )

        east = (
            range_m
            * math.sin(bearing)
        )

        target_lat = lat + math.degrees(
            north / EARTH_RADIUS_M
        )

        cos_lat = math.cos(
            math.radians(lat)
        )

        if abs(cos_lat) < 1e-6:
            return

        target_lon = lon + math.degrees(
            east
            / (
                EARTH_RADIUS_M
                * cos_lat
            )
        )

        target = {
            'mmsi': 999000001,
            'name': 'SIM_TARGET',
            'callsign': 'SIM001',
            'ship_type': None,
            'lat': target_lat,
            'lon': target_lon,
            'sog_knots': sog_knots,
            'cog_deg': cog_deg,
            'heading_deg': cog_deg,
            'state': 'ACTIVE',
            'position_age_s': 0.0,
        }

        msg = String()

        msg.data = json.dumps(
            [target],
            separators=(',', ':'),
        )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AISTestTarget()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
