#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node

import serial
from pyais import decode

from std_msgs.msg import String, Bool, UInt16


POSITION_TYPES = {1, 2, 3, 18, 19}


def clean_heading(value):
    if value is None:
        return None

    value = float(value)

    if 0.0 <= value <= 359.0:
        return value

    return None


def clean_course(value):
    if value is None:
        return None

    value = float(value)

    if 0.0 <= value < 360.0:
        return value

    return None


def clean_speed(value):
    if value is None:
        return None

    value = float(value)

    if 0.0 <= value < 102.3:
        return value

    return None


def clean_text(value):
    if value is None:
        return None

    value = str(value).replace("@", "").strip()

    return value if value else None


class AisNode(Node):

    def __init__(self):
        super().__init__("ais_node")

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        self.declare_parameter(
            "port",
            "/dev/serial/by-id/"
            "usb-1a86_USB_Serial-if00-port0",
        )

        self.declare_parameter("baud", 38400)

        self.declare_parameter(
            "reconnect_interval_s",
            2.0,
        )

        self.declare_parameter(
            "active_timeout_s",
            60.0,
        )

        self.declare_parameter(
            "remove_timeout_s",
            300.0,
        )

        self.declare_parameter(
            "publish_rate_hz",
            1.0,
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

        self.reconnect_interval_s = (
            self.get_parameter(
                "reconnect_interval_s"
            )
            .get_parameter_value()
            .double_value
        )

        self.active_timeout_s = (
            self.get_parameter(
                "active_timeout_s"
            )
            .get_parameter_value()
            .double_value
        )

        self.remove_timeout_s = (
            self.get_parameter(
                "remove_timeout_s"
            )
            .get_parameter_value()
            .double_value
        )

        publish_rate_hz = (
            self.get_parameter(
                "publish_rate_hz"
            )
            .get_parameter_value()
            .double_value
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.targets_pub = self.create_publisher(
            String,
            "/ais/targets",
            10,
        )

        # Legacy topic: means AIS message activity, NOT serial state.
        self.connected_pub = self.create_publisher(
            Bool,
            "/ais/connected",
            10,
        )

        self.serial_connected_pub = self.create_publisher(
            Bool,
            "/ais/serial_connected",
            10,
        )

        self.rx_active_pub = self.create_publisher(
            Bool,
            "/ais/rx_active",
            10,
        )

        self.count_pub = self.create_publisher(
            UInt16,
            "/ais/target_count",
            10,
        )

        # ----------------------------------------------------
        # State
        # ----------------------------------------------------

        self.ser = None
        self.buffer = bytearray()

        self.vessels = {}

        self.last_connect_attempt = 0.0
        self.last_message_time = None

        self.multipart = {}

        # Fast serial polling
        self.serial_timer = self.create_timer(
            0.02,
            self.poll_serial,
        )

        # Slower state publication
        self.publish_timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_targets,
        )

        self.get_logger().info(
            "AIS node started"
        )

    # ========================================================
    # Serial
    # ========================================================

    def connect_serial(self):

        now = time.monotonic()

        if (
            now - self.last_connect_attempt
            < self.reconnect_interval_s
        ):
            return

        self.last_connect_attempt = now

        try:

            self.get_logger().info(
                f"Connecting to AIS "
                f"{self.port} @ {self.baud}"
            )

            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
            )

            self.ser.reset_input_buffer()

            self.get_logger().info(
                "AIS connected"
            )

        except Exception as exc:

            self.ser = None

            self.get_logger().warning(
                f"AIS connection failed: {exc}"
            )

    # ========================================================
    # Vessel cache
    # ========================================================

    def get_vessel(self, mmsi):

        if mmsi not in self.vessels:

            self.vessels[mmsi] = {
                "mmsi": int(mmsi),

                "name": None,
                "callsign": None,
                "ship_type": None,

                "lat": None,
                "lon": None,

                "sog_knots": None,
                "cog_deg": None,
                "heading_deg": None,

                "last_seen": None,
                "last_position_time": None,
            }

        return self.vessels[mmsi]

    # ========================================================
    # AIS processing
    # ========================================================

    def process_message(self, msg):

        data = msg.asdict()

        msg_type = data.get("msg_type")
        mmsi = data.get("mmsi")

        if mmsi is None:
            return

        now = time.monotonic()

        vessel = self.get_vessel(mmsi)

        vessel["last_seen"] = now

        # ----------------------------------------------------
        # Position
        # ----------------------------------------------------

        if msg_type in POSITION_TYPES:

            lat = data.get("lat")
            lon = data.get("lon")

            if (
                lat is not None
                and lon is not None
                and -90 <= lat <= 90
                and -180 <= lon <= 180
            ):

                vessel["lat"] = float(lat)
                vessel["lon"] = float(lon)

                vessel[
                    "last_position_time"
                ] = now

            vessel["sog_knots"] = (
                clean_speed(
                    data.get("speed")
                )
            )

            vessel["cog_deg"] = (
                clean_course(
                    data.get("course")
                )
            )

            vessel["heading_deg"] = (
                clean_heading(
                    data.get("heading")
                )
            )

        # ----------------------------------------------------
        # Class A static data
        # ----------------------------------------------------

        elif msg_type == 5:

            vessel["name"] = clean_text(
                data.get("shipname")
            )

            vessel["callsign"] = clean_text(
                data.get("callsign")
            )

            vessel["ship_type"] = (
                data.get("ship_type")
            )

        # ----------------------------------------------------
        # Class B static data
        # ----------------------------------------------------

        elif msg_type == 24:

            name = clean_text(
                data.get("shipname")
            )

            callsign = clean_text(
                data.get("callsign")
            )

            if name:
                vessel["name"] = name

            if callsign:
                vessel["callsign"] = callsign

            if (
                data.get("ship_type")
                is not None
            ):
                vessel["ship_type"] = (
                    data.get("ship_type")
                )

    def decode_line(self, line):

        parts = line.split(",")

        if len(parts) < 7:
            return

        try:
            total = int(parts[1])
            number = int(parts[2])

        except ValueError:
            return

        # Single fragment
        if total == 1:

            msg = decode(line)

            self.process_message(msg)

            return

        sequence_id = parts[3]
        channel = parts[4]

        key = (
            sequence_id,
            channel,
            total,
        )

        if key not in self.multipart:

            self.multipart[key] = {
                "time": time.monotonic(),
                "parts": {},
            }

        self.multipart[key]["parts"][
            number
        ] = line

        entry = self.multipart[key]

        if len(entry["parts"]) == total:

            fragments = [
                entry["parts"][i]
                for i in range(
                    1,
                    total + 1,
                )
            ]

            del self.multipart[key]

            msg = decode(*fragments)

            self.process_message(msg)

    # ========================================================
    # Serial polling
    # ========================================================

    def poll_serial(self):

        if self.ser is None:

            self.connect_serial()

            return

        try:

            waiting = self.ser.in_waiting

            if waiting <= 0:
                return

            data = self.ser.read(
                waiting
            )

            self.buffer.extend(data)

            while b"\n" in self.buffer:

                raw_line, _, rest = (
                    self.buffer.partition(
                        b"\n"
                    )
                )

                self.buffer = bytearray(
                    rest
                )

                line = raw_line.decode(
                    "ascii",
                    errors="ignore",
                ).strip()

                if not (
                    line.startswith(
                        "!AIVDM"
                    )
                    or line.startswith(
                        "!AIVDO"
                    )
                ):
                    continue

                self.last_message_time = (
                    time.monotonic()
                )

                try:

                    self.decode_line(line)

                except Exception as exc:

                    self.get_logger().warning(
                        f"AIS decode error: "
                        f"{exc}"
                    )

        except Exception as exc:

            self.get_logger().error(
                f"AIS serial error: {exc}"
            )

            try:
                self.ser.close()
            except Exception:
                pass

            self.ser = None

    # ========================================================
    # Publication
    # ========================================================

    def publish_targets(self):

        now = time.monotonic()

        # Remove expired vessels
        expired = []

        for mmsi, vessel in (
            self.vessels.items()
        ):

            last_seen = vessel[
                "last_seen"
            ]

            if (
                last_seen is not None
                and now - last_seen
                > self.remove_timeout_s
            ):
                expired.append(mmsi)

        for mmsi in expired:
            del self.vessels[mmsi]

        targets = []

        for vessel in (
            self.vessels.values()
        ):

            output = vessel.copy()

            last_position = vessel[
                "last_position_time"
            ]

            if last_position is None:

                age = None
                state = "NO_POSITION"

            else:

                age = (
                    now
                    - last_position
                )

                if (
                    age
                    <= self.active_timeout_s
                ):
                    state = "ACTIVE"

                else:
                    state = "STALE"

            output["state"] = state
            output["position_age_s"] = age

            # monotonic timestamps are not
            # useful outside this process
            output.pop("last_seen")
            output.pop(
                "last_position_time"
            )

            targets.append(output)

        msg = String()

        msg.data = json.dumps(
            targets,
            separators=(",", ":"),
        )

        self.targets_pub.publish(msg)

        count_msg = UInt16()
        count_msg.data = len(targets)

        self.count_pub.publish(
            count_msg
        )

        # Serial layer: can the process currently access an open port?
        serial_connected = (
            self.ser is not None
            and getattr(self.ser, "is_open", False)
        )

        serial_msg = Bool()
        serial_msg.data = serial_connected

        self.serial_connected_pub.publish(
            serial_msg
        )

        # RF/message layer: have we recently received AIS traffic?
        if self.last_message_time is None:

            rx_active = False

        else:

            rx_active = (
                now
                - self.last_message_time
                < 30.0
            )

        rx_msg = Bool()
        rx_msg.data = rx_active

        self.rx_active_pub.publish(
            rx_msg
        )

        # Backward compatibility: /ais/connected retains its old
        # meaning (recent AIS traffic).
        connected_msg = Bool()
        connected_msg.data = rx_active

        self.connected_pub.publish(
            connected_msg
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

    node = AisNode()

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
