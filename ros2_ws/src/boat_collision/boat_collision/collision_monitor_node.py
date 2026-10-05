#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, String

from .colreg import classify_encounter


EARTH_RADIUS_M = 6371000.0
KNOT_TO_MPS = 0.514444


class CollisionMonitor(Node):

    def __init__(self):
        super().__init__('collision_monitor')

        # --------------------------------------------------
        # Operating mode
        # --------------------------------------------------
        self.declare_parameter('test_mode', True)

        # Desktop test scenario
        self.declare_parameter('test_own_sog_mps', 1.0)
        self.declare_parameter('test_own_cog_deg', 0.0)

        self.declare_parameter('test_target_range_m', 100.0)
        self.declare_parameter('test_target_bearing_deg', 0.0)
        self.declare_parameter('test_target_sog_mps', 0.0)
        self.declare_parameter('test_target_cog_deg', 0.0)

        # Advisory-only thresholds
        self.declare_parameter('warn_cpa_m', 50.0)
        self.declare_parameter('warn_tcpa_s', 300.0)

        # Input freshness
        self.declare_parameter('ownship_stale_s', 6.0)
        self.declare_parameter('ais_stale_s', 5.0)
        self.declare_parameter('ais_topic', '/ais/targets')

        # Suppress stationary GPS velocity noise
        self.declare_parameter(
            'ownship_min_speed_mps',
            0.5,
        )

        # --------------------------------------------------
        # State
        # --------------------------------------------------
        self.own_fix = None
        self.own_fix_time = None

        self.own_velocity = None
        self.own_velocity_time = None

        self.own_heading = None
        self.own_heading_time = None

        self.ais_targets = []
        self.ais_time = None

        # --------------------------------------------------
        # Inputs
        # --------------------------------------------------
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.fix_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            self.velocity_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_cb,
            qos_profile_sensor_data,
        )

        ais_topic = self.get_parameter(
            'ais_topic'
        ).value

        self.create_subscription(
            String,
            ais_topic,
            self.ais_cb,
            10,
        )

        # --------------------------------------------------
        # Outputs
        # --------------------------------------------------
        self.status_pub = self.create_publisher(
            String,
            '/collision/status',
            10,
        )

        self.targets_pub = self.create_publisher(
            String,
            '/collision/targets',
            10,
        )

        self.risk_pub = self.create_publisher(
            Bool,
            '/collision/risk',
            10,
        )

        # These represent the highest-risk target.
        self.cpa_pub = self.create_publisher(
            Float64,
            '/collision/cpa_m',
            10,
        )

        self.tcpa_pub = self.create_publisher(
            Float64,
            '/collision/tcpa_s',
            10,
        )

        self.create_timer(
            2.0,
            self.evaluate,
        )

        self.get_logger().info(
            'Collision monitor started '
            '(advisory only)'
        )

    @staticmethod
    def now():
        return time.monotonic()

    def fix_cb(self, msg):
        self.own_fix = msg
        self.own_fix_time = self.now()

    def velocity_cb(self, msg):
        self.own_velocity = msg
        self.own_velocity_time = self.now()

    def heading_cb(self, msg):
        self.own_heading = msg.data
        self.own_heading_time = self.now()

    def ais_cb(self, msg):
        self.ais_time = self.now()

        try:
            data = json.loads(msg.data)

            if isinstance(data, list):
                self.ais_targets = data
            else:
                self.ais_targets = []

        except Exception as exc:
            self.ais_targets = []

            self.get_logger().warning(
                f'AIS target JSON error: {exc}'
            )

    @staticmethod
    def velocity_from_sog_cog(
        sog_mps,
        cog_deg,
    ):
        """
        Navigation convention:
        0 deg = north
        90 deg = east
        """

        a = math.radians(cog_deg)

        east = sog_mps * math.sin(a)
        north = sog_mps * math.cos(a)

        return east, north

    @staticmethod
    def relative_position_test(
        range_m,
        bearing_deg,
    ):
        a = math.radians(bearing_deg)

        east = range_m * math.sin(a)
        north = range_m * math.cos(a)

        return east, north

    @staticmethod
    def relative_position_geo(
        own_lat,
        own_lon,
        target_lat,
        target_lon,
    ):
        """
        Local tangent-plane approximation.

        Accurate enough for local AIS collision
        monitoring over normal USV ranges.
        """

        lat1 = math.radians(own_lat)
        lat2 = math.radians(target_lat)

        dlat = lat2 - lat1
        dlon = math.radians(
            target_lon - own_lon
        )

        mean_lat = 0.5 * (
            lat1 + lat2
        )

        north = (
            EARTH_RADIUS_M
            * dlat
        )

        east = (
            EARTH_RADIUS_M
            * dlon
            * math.cos(mean_lat)
        )

        return east, north

    @staticmethod
    def wrap_180(angle_deg):
        return (
            (angle_deg + 180.0)
            % 360.0
        ) - 180.0

    @staticmethod
    def range_bearing(
        east,
        north,
    ):
        distance = math.hypot(
            east,
            north,
        )

        bearing = (
            math.degrees(
                math.atan2(
                    east,
                    north,
                )
            )
            % 360.0
        )

        return distance, bearing

    @staticmethod
    def cpa_tcpa(
        rx,
        ry,
        vx,
        vy,
    ):
        """
        r = target position relative to ownship
        v = target velocity relative to ownship
        """

        v2 = vx * vx + vy * vy

        if v2 < 1e-6:

            current_range = math.hypot(
                rx,
                ry,
            )

            return (
                current_range,
                math.nan,
            )

        tcpa = -(
            rx * vx
            +
            ry * vy
        ) / v2

        if tcpa < 0.0:

            cpa = math.hypot(
                rx,
                ry,
            )

        else:

            cx = rx + vx * tcpa
            cy = ry + vy * tcpa

            cpa = math.hypot(
                cx,
                cy,
            )

        return cpa, tcpa

    def risk_check(
        self,
        cpa_m,
        tcpa_s,
    ):

        warn_cpa = self.get_parameter(
            'warn_cpa_m'
        ).value

        warn_tcpa = self.get_parameter(
            'warn_tcpa_s'
        ).value

        return (
            math.isfinite(tcpa_s)
            and 0.0 <= tcpa_s <= warn_tcpa
            and cpa_m <= warn_cpa
        )

    def evaluate(self):

        if self.get_parameter(
            'test_mode'
        ).value:

            self.evaluate_test()

        else:

            self.evaluate_real()

    def evaluate_test(self):

        own_sog = self.get_parameter(
            'test_own_sog_mps'
        ).value

        own_cog = self.get_parameter(
            'test_own_cog_deg'
        ).value

        target_range = self.get_parameter(
            'test_target_range_m'
        ).value

        target_bearing = self.get_parameter(
            'test_target_bearing_deg'
        ).value

        target_sog = self.get_parameter(
            'test_target_sog_mps'
        ).value

        target_cog = self.get_parameter(
            'test_target_cog_deg'
        ).value

        own_e, own_n = (
            self.velocity_from_sog_cog(
                own_sog,
                own_cog,
            )
        )

        tgt_e, tgt_n = (
            self.velocity_from_sog_cog(
                target_sog,
                target_cog,
            )
        )

        rel_e, rel_n = (
            self.relative_position_test(
                target_range,
                target_bearing,
            )
        )

        rel_ve = tgt_e - own_e
        rel_vn = tgt_n - own_n

        cpa_m, tcpa_s = self.cpa_tcpa(
            rel_e,
            rel_n,
            rel_ve,
            rel_vn,
        )

        risk = self.risk_check(
            cpa_m,
            tcpa_s,
        )

        target = {
            'source': 'TEST',
            'range_m': round(
                target_range,
                2,
            ),
            'bearing_deg': round(
                target_bearing,
                2,
            ),
            'cpa_m': round(
                cpa_m,
                2,
            ),
            'tcpa_s': (
                round(tcpa_s, 2)
                if math.isfinite(tcpa_s)
                else None
            ),
            'risk': risk,
        }

        self.publish_results(
            mode='TEST',
            targets=[target],
        )

    def evaluate_real(self):

        now = self.now()

        own_stale = self.get_parameter(
            'ownship_stale_s'
        ).value

        ais_stale = self.get_parameter(
            'ais_stale_s'
        ).value

        # ----------------------------------------------
        # Ownship validation
        # ----------------------------------------------
        if (
            self.own_fix is None
            or self.own_fix_time is None
        ):
            self.publish_wait(
                'NO_OWNSHIP_POSITION'
            )
            return

        if (
            now - self.own_fix_time
            > own_stale
        ):
            self.publish_wait(
                'STALE_OWNSHIP_POSITION'
            )
            return

        if self.own_fix.status.status < 0:
            self.publish_wait(
                'NO_GPS_FIX'
            )
            return

        if (
            self.own_velocity is None
            or self.own_velocity_time is None
            or
            now - self.own_velocity_time
            > own_stale
        ):
            self.publish_wait(
                'NO_OWNSHIP_VELOCITY'
            )
            return

        # ----------------------------------------------
        # AIS validation
        # ----------------------------------------------
        if self.ais_time is None:
            self.publish_wait(
                'NO_AIS_DATA'
            )
            return

        if (
            now - self.ais_time
            > ais_stale
        ):
            self.publish_wait(
                'STALE_AIS_DATA'
            )
            return

        own_lat = self.own_fix.latitude
        own_lon = self.own_fix.longitude

        own_e = (
            self.own_velocity
            .twist
            .linear
            .x
        )

        own_n = (
            self.own_velocity
            .twist
            .linear
            .y
        )

        own_speed = math.hypot(
            own_e,
            own_n,
        )

        own_heading = None

        if (
            self.own_heading is not None
            and self.own_heading_time is not None
            and now - self.own_heading_time
            <= own_stale
            and math.isfinite(self.own_heading)
        ):
            own_heading = (
                self.own_heading
                % 360.0
            )

        min_speed = self.get_parameter(
            'ownship_min_speed_mps'
        ).value

        # Suppress GPS drift while stationary.
        if own_speed < min_speed:
            own_e = 0.0
            own_n = 0.0

        results = []

        # ----------------------------------------------
        # Process each AIS target
        # ----------------------------------------------
        for vessel in self.ais_targets:

            if (
                vessel.get('state')
                != 'ACTIVE'
            ):
                continue

            lat = vessel.get('lat')
            lon = vessel.get('lon')

            if (
                lat is None
                or lon is None
            ):
                continue

            try:
                lat = float(lat)
                lon = float(lon)

            except (TypeError, ValueError):
                continue

            rel_e, rel_n = (
                self.relative_position_geo(
                    own_lat,
                    own_lon,
                    lat,
                    lon,
                )
            )

            distance, bearing = (
                self.range_bearing(
                    rel_e,
                    rel_n,
                )
            )

            sog_knots = vessel.get(
                'sog_knots'
            )

            cog_deg = vessel.get(
                'cog_deg'
            )

            relative_bearing = None

            if own_heading is not None:
                relative_bearing = self.wrap_180(
                    bearing - own_heading
                )

            result = {
                'mmsi': vessel.get('mmsi'),
                'name': vessel.get('name'),
                'state': vessel.get('state'),
                'position_age_s':
                    vessel.get(
                        'position_age_s'
                    ),
                'range_m':
                    round(distance, 2),
                'bearing_deg':
                    round(bearing, 2),
                'own_heading_deg':
                    (
                        round(own_heading, 2)
                        if own_heading is not None
                        else None
                    ),
                'relative_bearing_deg':
                    (
                        round(relative_bearing, 2)
                        if relative_bearing is not None
                        else None
                    ),
                'sog_knots':
                    sog_knots,
                'cog_deg':
                    cog_deg,
                'heading_deg':
                    vessel.get('heading_deg'),
            }

            # No reliable target velocity:
            # report range, but do not invent CPA.
            if (
                sog_knots is None
                or cog_deg is None
            ):

                result.update({
                    'status':
                        'NO_VELOCITY',
                    'cpa_m': None,
                    'tcpa_s': None,
                    'risk': False,
                })

                results.append(
                    result
                )

                continue

            try:
                target_sog_mps = (
                    float(sog_knots)
                    * KNOT_TO_MPS
                )

                target_cog_deg = float(
                    cog_deg
                )

            except (
                TypeError,
                ValueError,
            ):
                continue

            tgt_e, tgt_n = (
                self.velocity_from_sog_cog(
                    target_sog_mps,
                    target_cog_deg,
                )
            )

            rel_ve = tgt_e - own_e
            rel_vn = tgt_n - own_n

            cpa_m, tcpa_s = (
                self.cpa_tcpa(
                    rel_e,
                    rel_n,
                    rel_ve,
                    rel_vn,
                )
            )

            risk = self.risk_check(
                cpa_m,
                tcpa_s,
            )

            target_heading = vessel.get(
                'heading_deg'
            )

            colreg = classify_encounter(
                own_heading_deg=own_heading,
                target_bearing_deg=bearing,
                target_heading_deg=target_heading,
                target_cog_deg=target_cog_deg,
                own_speed_mps=own_speed,
                target_speed_mps=target_sog_mps,
            )

            result.update({
                'encounter':
                    colreg['encounter'],
                'colreg_role':
                    colreg['role'],
                'colreg_action':
                    colreg['action'],
                'status':
                    'WARN'
                    if risk
                    else 'CLEAR',
                'cpa_m':
                    round(cpa_m, 2),
                'tcpa_s':
                    (
                        round(tcpa_s, 2)
                        if math.isfinite(
                            tcpa_s
                        )
                        else None
                    ),
                'risk': risk,
            })

            results.append(
                result
            )

        self.publish_results(
            mode='REAL',
            targets=results,
        )

    def publish_wait(
        self,
        reason,
    ):

        summary = {
            'mode': 'REAL',
            'status': 'WAIT',
            'reason': reason,
            'targets': 0,
            'risk_targets': 0,
        }

        msg = String()
        msg.data = json.dumps(
            summary,
            separators=(',', ':'),
        )

        self.status_pub.publish(
            msg
        )

        risk_msg = Bool()
        risk_msg.data = False

        self.risk_pub.publish(
            risk_msg
        )

        self.get_logger().info(
            f'WAIT  {reason}'
        )

    def publish_results(
        self,
        mode,
        targets,
    ):

        risk_targets = [
            target
            for target in targets
            if target.get('risk', False)
        ]

        any_risk = (
            len(risk_targets) > 0
        )

        summary = {
            'mode': mode,
            'status':
                'WARN'
                if any_risk
                else 'CLEAR',
            'targets':
                len(targets),
            'risk_targets':
                len(risk_targets),
        }

        status_msg = String()
        status_msg.data = json.dumps(
            summary,
            separators=(',', ':'),
        )

        self.status_pub.publish(
            status_msg
        )

        targets_msg = String()
        targets_msg.data = json.dumps(
            targets,
            separators=(',', ':'),
        )

        self.targets_pub.publish(
            targets_msg
        )

        risk_msg = Bool()
        risk_msg.data = any_risk

        self.risk_pub.publish(
            risk_msg
        )

        # Highest-risk target first.
        valid = [
            t for t in targets
            if t.get('cpa_m') is not None
        ]

        if valid:

            valid.sort(
                key=lambda t: (
                    not t.get(
                        'risk',
                        False,
                    ),
                    t.get(
                        'cpa_m',
                        float('inf'),
                    ),
                )
            )

            primary = valid[0]

            cpa_msg = Float64()
            cpa_msg.data = float(
                primary['cpa_m']
            )

            self.cpa_pub.publish(
                cpa_msg
            )

            tcpa_msg = Float64()

            if (
                primary.get('tcpa_s')
                is None
            ):
                tcpa_msg.data = math.nan
            else:
                tcpa_msg.data = float(
                    primary['tcpa_s']
                )

            self.tcpa_pub.publish(
                tcpa_msg
            )

        self.get_logger().info(
            f'{summary["status"]}  '
            f'mode={mode}  '
            f'targets={len(targets)}  '
            f'risk={len(risk_targets)}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CollisionMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
