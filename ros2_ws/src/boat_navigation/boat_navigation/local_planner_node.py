#!/usr/bin/env python3

import math
import statistics
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class LocalPlannerNode(Node):

    def __init__(self):
        super().__init__('local_planner_node')

        self.declare_parameter('front_angle_deg', 0.0)

        # Bench-test geometry. Replace with real boat geometry later.
        self.declare_parameter('corridor_start_x_m', 0.10)
        self.declare_parameter('corridor_half_width_m', 0.15)
        self.declare_parameter('lookahead_m', 3.0)

        self.declare_parameter('min_safe_clearance_m', 0.80)
        self.declare_parameter('nominal_target_distance_m', 2.0)

        self.declare_parameter('candidate_step_deg', 15.0)
        self.declare_parameter('max_turn_deg', 75.0)

        self.declare_parameter('min_cluster_points', 8)
        self.declare_parameter('max_index_gap', 3)
        self.declare_parameter('max_range_jump_m', 0.30)

        # Penalize unnecessarily large turns.
        self.declare_parameter('turn_penalty_m_per_deg', 0.01)

        self.front_angle_deg = float(
            self.get_parameter('front_angle_deg').value
        )

        self.start_x = float(
            self.get_parameter('corridor_start_x_m').value
        )

        self.half_width = float(
            self.get_parameter('corridor_half_width_m').value
        )

        self.lookahead = float(
            self.get_parameter('lookahead_m').value
        )

        self.min_safe_clearance = float(
            self.get_parameter('min_safe_clearance_m').value
        )

        self.nominal_target_distance = float(
            self.get_parameter('nominal_target_distance_m').value
        )

        self.step_deg = float(
            self.get_parameter('candidate_step_deg').value
        )

        self.max_turn_deg = float(
            self.get_parameter('max_turn_deg').value
        )

        self.min_cluster_points = int(
            self.get_parameter('min_cluster_points').value
        )

        self.max_index_gap = int(
            self.get_parameter('max_index_gap').value
        )

        self.max_range_jump = float(
            self.get_parameter('max_range_jump_m').value
        )

        self.turn_penalty = float(
            self.get_parameter('turn_penalty_m_per_deg').value
        )

        self.action = 'KEEP_COURSE'
        self.action_time = 0.0

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.action_sub = self.create_subscription(
            String,
            '/navigation/avoidance/action',
            self.action_callback,
            10
        )

        self.valid_pub = self.create_publisher(
            Bool,
            '/navigation/local_planner/valid',
            10
        )

        self.angle_pub = self.create_publisher(
            Float32,
            '/navigation/local_planner/angle_deg',
            10
        )

        self.clearance_pub = self.create_publisher(
            Float32,
            '/navigation/local_planner/clearance_m',
            10
        )

        self.x_pub = self.create_publisher(
            Float32,
            '/navigation/local_planner/x_forward_m',
            10
        )

        self.y_pub = self.create_publisher(
            Float32,
            '/navigation/local_planner/y_right_m',
            10
        )

        self.last_log = 0.0

        self.get_logger().info(
            'Local planner started -- planning only'
        )

    def action_callback(self, msg):
        self.action = msg.data
        self.action_time = time.monotonic()

    def candidate_clearance(
        self,
        points,
        candidate_deg,
    ):
        """
        Candidate-frame rectangular corridor.

        Boat frame:
          +x = forward
          +y = left

        candidate_deg:
          + = left
          - = right
        """

        theta = math.radians(candidate_deg)

        c = math.cos(theta)
        s = math.sin(theta)

        inside = []

        for index, x, y, radial in points:

            # Rotate boat-frame point into candidate frame.
            xc = x * c + y * s
            yc = -x * s + y * c

            if xc <= self.start_x:
                continue

            clearance = xc - self.start_x

            if clearance > self.lookahead:
                continue

            if abs(yc) > self.half_width:
                continue

            inside.append(
                (index, radial, clearance)
            )

        if not inside:
            return math.inf

        clusters = []
        current = [inside[0]]

        for point in inside[1:]:

            prev = current[-1]

            index_gap = point[0] - prev[0]
            range_jump = abs(
                point[1] - prev[1]
            )

            if (
                index_gap <= self.max_index_gap
                and range_jump <= self.max_range_jump
            ):
                current.append(point)
            else:
                clusters.append(current)
                current = [point]

        clusters.append(current)

        clusters = [
            cluster
            for cluster in clusters
            if len(cluster) >= self.min_cluster_points
        ]

        if not clusters:
            return math.inf

        obstacle_clearances = []

        for cluster in clusters:

            obstacle_clearances.append(
                statistics.median(
                    p[2] for p in cluster
                )
            )

        return min(obstacle_clearances)

    def publish_invalid(self):
        valid = Bool()
        valid.data = False
        self.valid_pub.publish(valid)

    def scan_callback(self, msg):

        if self.action not in (
            'TURN_LEFT',
            'TURN_RIGHT',
        ):
            self.publish_invalid()
            return

        points = []

        for i, r in enumerate(msg.ranges):

            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            lidar_angle = math.degrees(
                msg.angle_min
                + i * msg.angle_increment
            )

            relative_angle = wrap_deg(
                lidar_angle
                - self.front_angle_deg
            )

            theta = math.radians(
                relative_angle
            )

            # Boat frame:
            # +x forward
            # +y left
            x = r * math.cos(theta)
            y = r * math.sin(theta)

            points.append(
                (i, x, y, r)
            )

        # Positive = LEFT
        # Negative = RIGHT
        sign = (
            1.0
            if self.action == 'TURN_LEFT'
            else -1.0
        )

        candidates = []

        angle = self.step_deg

        while angle <= self.max_turn_deg + 1e-6:

            candidate_angle = (
                sign * angle
            )

            clearance = self.candidate_clearance(
                points,
                candidate_angle,
            )

            if (
                math.isinf(clearance)
                or clearance >= self.min_safe_clearance
            ):

                effective_clearance = (
                    self.lookahead
                    if math.isinf(clearance)
                    else min(
                        clearance,
                        self.lookahead
                    )
                )

                score = (
                    effective_clearance
                    - self.turn_penalty
                    * abs(candidate_angle)
                )

                candidates.append(
                    (
                        score,
                        candidate_angle,
                        clearance,
                    )
                )

            angle += self.step_deg

        if not candidates:
            self.publish_invalid()

            if (
                time.monotonic()
                - self.last_log
                >= 1.0
            ):
                self.last_log = time.monotonic()

                self.get_logger().warning(
                    f'NO SAFE LOCAL PATH | '
                    f'action={self.action}'
                )

            return

        _, best_angle, clearance = max(
            candidates,
            key=lambda x: x[0]
        )

        # Never put the target beyond the measured obstacle.
        if math.isinf(clearance):

            target_distance = (
                self.nominal_target_distance
            )

        else:

            target_distance = min(
                self.nominal_target_distance,
                0.80 * clearance,
            )

        target_distance = max(
            target_distance,
            0.50,
        )

        theta = math.radians(
            best_angle
        )

        x_forward = (
            target_distance
            * math.cos(theta)
        )

        # planner angle +LEFT,
        # Guided BODY_OFFSET uses +RIGHT.
        y_right = (
            -target_distance
            * math.sin(theta)
        )

        valid_msg = Bool()
        valid_msg.data = True

        angle_msg = Float32()
        angle_msg.data = float(
            best_angle
        )

        clearance_msg = Float32()
        clearance_msg.data = float(
            clearance
        )

        x_msg = Float32()
        x_msg.data = float(
            x_forward
        )

        y_msg = Float32()
        y_msg.data = float(
            y_right
        )

        self.valid_pub.publish(
            valid_msg
        )

        self.angle_pub.publish(
            angle_msg
        )

        self.clearance_pub.publish(
            clearance_msg
        )

        self.x_pub.publish(
            x_msg
        )

        self.y_pub.publish(
            y_msg
        )

        now = time.monotonic()

        if now - self.last_log >= 0.5:

            self.last_log = now

            clearance_text = (
                'inf'
                if math.isinf(clearance)
                else f'{clearance:.2f}m'
            )

            self.get_logger().info(
                f'{self.action} | '
                f'best_angle={best_angle:+.0f}deg | '
                f'clearance={clearance_text} | '
                f'target=({x_forward:.2f}, '
                f'{y_right:.2f})m'
            )


def main(args=None):

    rclpy.init(args=args)

    node = LocalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
