#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

import serial

from sensor_msgs.msg import Range
from std_msgs.msg import Float32, UInt16


class TfminiNode(Node):

    def __init__(self):
        super().__init__("tfmini_node")

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        self.declare_parameter("port", "/dev/ttyAMA3")
        self.declare_parameter("baud", 115200)

        self.declare_parameter(
            "frame_id",
            "tfmini_link",
        )

        self.declare_parameter(
            "min_range",
            0.10,
        )

        self.declare_parameter(
            "max_range",
            12.0,
        )

        self.declare_parameter(
            "field_of_view_deg",
            3.6,
        )

        self.port = (
            self.get_parameter("port")
            .get_parameter_value()
            .string_value
        )

        self.baud = (
            self.get_parameter("baud")
            .get_parameter_value()
            .integer_value
        )

        self.frame_id = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )

        self.min_range = (
            self.get_parameter("min_range")
            .get_parameter_value()
            .double_value
        )

        self.max_range = (
            self.get_parameter("max_range")
            .get_parameter_value()
            .double_value
        )

        field_of_view_deg = (
            self.get_parameter("field_of_view_deg")
            .get_parameter_value()
            .double_value
        )

        self.field_of_view = math.radians(
            field_of_view_deg
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.range_pub = self.create_publisher(
            Range,
            "/sensors/tfmini/range",
            10,
        )

        self.strength_pub = self.create_publisher(
            UInt16,
            "/sensors/tfmini/strength",
            10,
        )

        self.temperature_pub = self.create_publisher(
            Float32,
            "/sensors/tfmini/internal_temperature",
            10,
        )

        # ----------------------------------------------------
        # Serial state
        # ----------------------------------------------------

        self.ser = None
        self.buffer = bytearray()

        self.last_connect_attempt = 0.0
        self.reconnect_interval_s = 2.0

        # Poll UART at 50 Hz.
        self.timer = self.create_timer(
            0.02,
            self.poll_serial,
        )

        self.get_logger().info(
            "TFmini node started"
        )

    # ========================================================
    # Serial connection
    # ========================================================

    def open_serial(self):

        now = time.monotonic()

        if (
            now - self.last_connect_attempt
            < self.reconnect_interval_s
        ):
            return

        self.last_connect_attempt = now

        try:

            self.get_logger().info(
                f"Connecting to {self.port} "
                f"@ {self.baud}"
            )

            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
            )

            self.ser.reset_input_buffer()

            self.buffer.clear()

            self.get_logger().info(
                "TFmini connected"
            )

        except Exception as exc:

            self.ser = None

            self.get_logger().warning(
                f"TFmini connection failed: {exc}"
            )

    # ========================================================
    # TFmini protocol
    # ========================================================

    @staticmethod
    def parse_frame(frame: bytes):

        if len(frame) != 9:
            return None

        if (
            frame[0] != 0x59
            or frame[1] != 0x59
        ):
            return None

        checksum = (
            sum(frame[:8])
            & 0xFF
        )

        if checksum != frame[8]:
            return None

        distance_cm = (
            frame[2]
            | (frame[3] << 8)
        )

        strength = (
            frame[4]
            | (frame[5] << 8)
        )

        temperature_raw = (
            frame[6]
            | (frame[7] << 8)
        )

        temperature_c = (
            temperature_raw / 8.0
            - 256.0
        )

        distance_m = (
            distance_cm / 100.0
        )

        return (
            distance_m,
            strength,
            temperature_c,
        )

    # ========================================================
    # Serial polling
    # ========================================================

    def poll_serial(self):

        if self.ser is None:

            self.open_serial()

            return

        try:

            waiting = self.ser.in_waiting

            if waiting <= 0:
                return

            chunk = self.ser.read(
                waiting
            )

            self.buffer.extend(chunk)

            while len(self.buffer) >= 9:

                header_index = (
                    self.buffer.find(
                        b"\x59\x59"
                    )
                )

                if header_index < 0:

                    if (
                        self.buffer[-1:]
                        == b"\x59"
                    ):
                        self.buffer[:] = (
                            self.buffer[-1:]
                        )

                    else:
                        self.buffer.clear()

                    break

                if header_index > 0:

                    del self.buffer[
                        :header_index
                    ]

                if len(self.buffer) < 9:
                    break

                frame = bytes(
                    self.buffer[:9]
                )

                parsed = (
                    self.parse_frame(
                        frame
                    )
                )

                if parsed is None:

                    del self.buffer[0]

                    continue

                del self.buffer[:9]

                (
                    distance_m,
                    strength,
                    temperature_c,
                ) = parsed

                self.publish_measurement(
                    distance_m,
                    strength,
                    temperature_c,
                )

        except Exception as exc:

            self.get_logger().error(
                f"TFmini serial error: {exc}"
            )

            try:
                self.ser.close()
            except Exception:
                pass

            self.ser = None

    # ========================================================
    # ROS publication
    # ========================================================

    def publish_measurement(
        self,
        distance_m,
        strength,
        temperature_c,
    ):

        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # Standard ROS Range message
        range_msg = Range()

        range_msg.header.stamp = stamp

        range_msg.header.frame_id = (
            self.frame_id
        )

        range_msg.radiation_type = (
            Range.INFRARED
        )

        range_msg.field_of_view = (
            self.field_of_view
        )

        range_msg.min_range = (
            self.min_range
        )

        range_msg.max_range = (
            self.max_range
        )

        if (
            self.min_range
            <= distance_m
            <= self.max_range
        ):
            range_msg.range = (
                distance_m
            )
        else:
            range_msg.range = (
                float("nan")
            )

        self.range_pub.publish(
            range_msg
        )

        # Signal strength
        strength_msg = UInt16()

        strength_msg.data = int(
            strength
        )

        self.strength_pub.publish(
            strength_msg
        )

        # Internal electronics temperature
        temperature_msg = Float32()

        temperature_msg.data = float(
            temperature_c
        )

        self.temperature_pub.publish(
            temperature_msg
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        if self.ser is not None:

            try:
                self.ser.close()

            except Exception:
                pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = TfminiNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

