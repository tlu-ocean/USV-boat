#!/usr/bin/env python3

from collections import deque
import json
import math
from statistics import median
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Range, NavSatFix
from std_msgs.msg import Bool, UInt16, UInt32, Float64, String
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, SysStatus


PROX_BIT = 1 << 8


class BoatHealthNode(Node):

    def __init__(self):
        super().__init__('boat_health')

        self.state = None
        self.state_time = None

        self.sys_status = None
        self.sys_time = None

        self.scan_times = deque(maxlen=50)
        self.scan_time = None
        self.scan_msg = None

        self.tfmini_times = deque(maxlen=100)
        self.tfmini_time = None
        self.tfmini_range = None

        self.ais_serial_connected = False
        self.ais_rx_active = False
        self.ais_target_count = 0

        self.ais_serial_time = None
        self.ais_rx_time = None
        self.ais_count_time = None

        self.collision_status = None
        self.collision_time = None

        # Ownship navigation state
        self.gps_fix = None
        self.gps_fix_time = None

        self.gps_satellites = None
        self.gps_sat_time = None

        self.heading = None
        self.heading_time = None

        self.gps_velocity = None
        self.gps_velocity_time = None

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10,
        )

        self.create_subscription(
            SysStatus,
            '/mavros/sys_status',
            self.sys_cb,
            10,
        )

        self.create_subscription(
            LaserScan,
            '/mavros/obstacle/send',
            self.scan_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Range,
            '/sensors/tfmini/range',
            self.tfmini_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Bool,
            '/ais/serial_connected',
            self.ais_serial_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/ais/rx_active',
            self.ais_rx_cb,
            10,
        )

        self.create_subscription(
            UInt16,
            '/ais/target_count',
            self.ais_count_cb,
            10,
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_fix_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            UInt32,
            '/mavros/global_position/raw/satellites',
            self.gps_sat_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            self.gps_velocity_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            '/collision/status',
            self.collision_cb,
            10,
        )

        self.create_timer(2.0, self.report)

        self.get_logger().info('Boat health monitor started')

    def now(self):
        return time.monotonic()

    def state_cb(self, msg):
        self.state = msg
        self.state_time = self.now()

    def sys_cb(self, msg):
        self.sys_status = msg
        self.sys_time = self.now()

    def scan_cb(self, msg):
        t = self.now()
        self.scan_time = t
        self.scan_times.append(t)
        self.scan_msg = msg

    def gps_fix_cb(self, msg):
        self.gps_fix = msg
        self.gps_fix_time = self.now()

    def gps_sat_cb(self, msg):
        self.gps_satellites = msg.data
        self.gps_sat_time = self.now()

    def heading_cb(self, msg):
        self.heading = msg.data
        self.heading_time = self.now()

    def gps_velocity_cb(self, msg):
        self.gps_velocity = msg
        self.gps_velocity_time = self.now()

    def collision_cb(self, msg):
        self.collision_time = self.now()

        try:
            self.collision_status = json.loads(msg.data)
        except Exception:
            self.collision_status = None

    def ais_serial_cb(self, msg):
        self.ais_serial_connected = msg.data
        self.ais_serial_time = self.now()

    def ais_rx_cb(self, msg):
        self.ais_rx_active = msg.data
        self.ais_rx_time = self.now()

    def ais_count_cb(self, msg):
        self.ais_target_count = msg.data
        self.ais_count_time = self.now()

    def tfmini_cb(self, msg):
        t = self.now()
        self.tfmini_time = t
        self.tfmini_times.append(t)
        self.tfmini_range = msg.range

    def tfmini_hz(self):
        if len(self.tfmini_times) < 2:
            return 0.0

        dt = self.tfmini_times[-1] - self.tfmini_times[0]

        if dt <= 0.0:
            return 0.0

        return (len(self.tfmini_times) - 1) / dt

    def s2_front_range(self):
        if self.scan_msg is None:
            return None

        msg = self.scan_msg

        # TFmini horizontal FOV is about 3.6 deg.
        # Compare against the central +/-2 deg of the S2 scan.
        half_width = math.radians(2.0)
        values = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) > half_width:
                continue

            if (
                math.isfinite(r)
                and msg.range_min <= r <= msg.range_max
            ):
                values.append(r)

        if not values:
            return None

        # Median is more robust than a single minimum return.
        return median(values)

    def scan_hz(self):
        if len(self.scan_times) < 2:
            return 0.0

        dt = self.scan_times[-1] - self.scan_times[0]

        if dt <= 0.0:
            return 0.0

        return (len(self.scan_times) - 1) / dt

    @staticmethod
    def age(now, stamp):
        if stamp is None:
            return float('inf')
        return now - stamp

    def report(self):
        now = self.now()

        # MAVROS / FCU
        state_age = self.age(now, self.state_time)

        if self.state is None:
            mavros_status = 'NO_DATA'
            fcu = 'UNKNOWN'
        elif state_age > 3.0:
            mavros_status = 'STALE'
            fcu = 'UNKNOWN'
        elif not self.state.connected:
            mavros_status = 'DISCONNECTED'
            fcu = 'UNKNOWN'
        else:
            mavros_status = 'OK'
            arm = 'ARMED' if self.state.armed else 'DISARMED'
            fcu = f'{self.state.mode} / {arm}'

        # LiDAR
        scan_age = self.age(now, self.scan_time)
        hz = self.scan_hz()

        if self.scan_time is None:
            lidar_status = 'NO_DATA'
        elif scan_age > 1.0:
            lidar_status = 'STALE'
        elif hz < 5.0:
            lidar_status = 'LOW_RATE'
        else:
            lidar_status = 'OK'

        # TFmini
        tfmini_age = self.age(now, self.tfmini_time)
        tfmini_hz = self.tfmini_hz()

        if self.tfmini_time is None:
            tfmini_status = 'NO_DATA'
        elif tfmini_age > 1.0:
            tfmini_status = 'STALE'
        elif tfmini_hz < 5.0:
            tfmini_status = 'LOW_RATE'
        else:
            tfmini_status = 'OK'

        # S2 vs TFmini forward cross-check
        s2_front = self.s2_front_range()

        if s2_front is None or self.tfmini_range is None:
            front_status = 'NO_DATA'
            front_text = 'S2=nan  TF=nan  delta=nan'
        else:
            delta = abs(s2_front - self.tfmini_range)

            # Monitoring-only initial tolerance:
            # at least 0.20 m or 15% of TFmini range.
            tolerance = max(0.20, 0.15 * self.tfmini_range)

            front_status = 'AGREE' if delta <= tolerance else 'MISMATCH'

            front_text = (
                f'S2={s2_front:.2f} m  '
                f'TF={self.tfmini_range:.2f} m  '
                f'delta={delta:.2f} m'
            )

        # Ownship / GPS
        gps_age = self.age(now, self.gps_fix_time)
        sat_age = self.age(now, self.gps_sat_time)
        heading_age = self.age(now, self.heading_time)
        velocity_age = self.age(now, self.gps_velocity_time)

        if self.gps_fix is None:
            gps_status = 'NO_DATA'
        elif gps_age > 6.0:
            gps_status = 'STALE'
        elif self.gps_fix.status.status < 0:
            gps_status = 'NO_FIX'
        else:
            gps_status = 'FIX'

        sats_text = (
            str(self.gps_satellites)
            if self.gps_satellites is not None and sat_age <= 3.0
            else '--'
        )

        if (
            self.heading is None
            or heading_age > 4.0
            or not math.isfinite(self.heading)
        ):
            heading_text = '---'
        else:
            heading_text = f'{self.heading:.1f} deg'

        speed = None
        cog = None

        if (
            self.gps_velocity is not None
            and velocity_age <= 6.0
        ):
            vx = self.gps_velocity.twist.linear.x
            vy = self.gps_velocity.twist.linear.y

            if math.isfinite(vx) and math.isfinite(vy):
                speed = math.hypot(vx, vy)

                # ENU: x=east, y=north.
                # Course clockwise from north.
                if speed >= 0.50:
                    cog = (
                        math.degrees(
                            math.atan2(vx, vy)
                        )
                        % 360.0
                    )

        speed_text = (
            f'{speed:.2f} m/s'
            if speed is not None
            else '---'
        )

        cog_text = (
            f'{cog:.1f} deg'
            if cog is not None
            else '--- (low speed)'
            if speed is not None
            else '---'
        )

        # AIS
        ais_age = max(
            self.age(now, self.ais_serial_time),
            self.age(now, self.ais_rx_time),
            self.age(now, self.ais_count_time),
        )

        if (
            self.ais_serial_time is None
            or self.ais_rx_time is None
            or self.ais_count_time is None
        ):
            ais_status = 'NO_DATA'
            ais_detail = 'unknown'
        elif ais_age > 5.0:
            ais_status = 'STALE'
            ais_detail = 'unknown'
        elif not self.ais_serial_connected:
            ais_status = 'SERIAL_DOWN'
            ais_detail = f'targets={self.ais_target_count}'
        else:
            ais_status = 'SERIAL_OK'
            rx_text = 'RX_ACTIVE' if self.ais_rx_active else 'RX_QUIET'
            ais_detail = f'{rx_text}  targets={self.ais_target_count}'

        # Collision advisory
        collision_age = self.age(
            now,
            self.collision_time,
        )

        if self.collision_status is None:
            collision_text = 'NO_DATA'

        elif collision_age > 5.0:
            collision_text = 'STALE'

        else:
            cstatus = self.collision_status.get(
                'status',
                'UNKNOWN',
            )

            targets = self.collision_status.get(
                'targets',
                0,
            )

            risks = self.collision_status.get(
                'risk_targets',
                0,
            )

            if cstatus == 'WAIT':
                reason = self.collision_status.get(
                    'reason',
                    'UNKNOWN',
                )

                collision_text = (
                    f'WAIT {reason}'
                )

            else:
                collision_text = (
                    f'{cstatus}  '
                    f'targets={targets}  '
                    f'risk={risks}'
                )

        # Pixhawk proximity
        sys_age = self.age(now, self.sys_time)

        if self.sys_status is None:
            prox_status = 'NO_DATA'
        elif sys_age > 4.0:
            prox_status = 'STALE'
        else:
            present = bool(self.sys_status.sensors_present & PROX_BIT)
            enabled = bool(self.sys_status.sensors_enabled & PROX_BIT)
            healthy = bool(self.sys_status.sensors_health & PROX_BIT)

            if present and enabled and healthy:
                prox_status = 'OK'
            elif not present:
                prox_status = 'NOT_PRESENT'
            elif not enabled:
                prox_status = 'DISABLED'
            else:
                prox_status = 'UNHEALTHY'

        self.get_logger().info(
            '\n'
            f'MAVROS  {mavros_status:<12} FCU {fcu}\n'
            f'GPS     {gps_status:<12} sats={sats_text}\n'
            f'HDG     {heading_text:<20} SPEED={speed_text}  COG={cog_text}\n'
            f'S2      {lidar_status:<12} {hz:5.1f} Hz  age={scan_age:4.2f}s\n'
            f'TFMINI  {tfmini_status:<12} {tfmini_hz:5.1f} Hz  '
            f'range={self.tfmini_range if self.tfmini_range is not None else float("nan"):.2f} m  '
            f'age={tfmini_age:4.2f}s\n'
            f'FRONT   {front_status:<12} {front_text}\n'
            f'PROX    {prox_status}\n'
            f'AIS     {ais_status:<12} {ais_detail}\n'
            f'COLL    {collision_text}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = BoatHealthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
