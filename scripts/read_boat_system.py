#!/usr/bin/env python3

import threading
import time

import serial
from pymavlink import mavutil


PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 115200

LIDAR_PORT = "/dev/ttyAMA3"
LIDAR_BAUD = 115200


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.Lock()

        self.mode = "UNKNOWN"
        self.armed = False

        self.latitude = None
        self.longitude = None
        self.heading_deg = None
        self.ground_speed_mps = None

        self.battery_voltage_v = None
        self.battery_remaining_pct = None

        self.lidar_distance_m = None
        self.lidar_strength = None
        self.lidar_temperature_c = None

        self.pixhawk_last_update = None
        self.lidar_last_update = None


def read_pixhawk(state: SharedState, stop_event: threading.Event) -> None:
    print(
        f"[Pixhawk] Opening {PIXHAWK_PORT} "
        f"at {PIXHAWK_BAUD} baud"
    )

    connection = mavutil.mavlink_connection(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        source_system=255,
    )

    connection.wait_heartbeat(timeout=15)

    print(
        "[Pixhawk] Connected:",
        f"system={connection.target_system}",
        f"component={connection.target_component}",
    )

    while not stop_event.is_set():
        message = connection.recv_match(blocking=True, timeout=1)

        if message is None:
            continue

        message_type = message.get_type()

        if message_type == "BAD_DATA":
            continue

        now = time.monotonic()

        with state.lock:
            if message_type == "HEARTBEAT":
                state.mode = mavutil.mode_string_v10(message)
                state.armed = bool(
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                state.pixhawk_last_update = now

            elif message_type == "GLOBAL_POSITION_INT":
                state.latitude = message.lat / 1e7
                state.longitude = message.lon / 1e7

                state.heading_deg = (
                    None
                    if message.hdg == 65535
                    else message.hdg / 100.0
                )

                state.pixhawk_last_update = now

            elif message_type == "VFR_HUD":
                state.ground_speed_mps = message.groundspeed
                state.pixhawk_last_update = now

            elif message_type == "SYS_STATUS":
                state.battery_voltage_v = (
                    None
                    if message.voltage_battery == 65535
                    else message.voltage_battery / 1000.0
                )

                state.battery_remaining_pct = (
                    None
                    if message.battery_remaining == -1
                    else message.battery_remaining
                )

                state.pixhawk_last_update = now


def read_lidar(state: SharedState, stop_event: threading.Event) -> None:
    print(
        f"[LiDAR] Opening {LIDAR_PORT} "
        f"at {LIDAR_BAUD} baud"
    )

    with serial.Serial(
        LIDAR_PORT,
        LIDAR_BAUD,
        timeout=0.5,
    ) as lidar:

        while not stop_event.is_set():
            first = lidar.read(1)

            if first != b"\x59":
                continue

            second = lidar.read(1)

            if second != b"\x59":
                continue

            payload = lidar.read(7)

            if len(payload) != 7:
                continue

            frame = b"\x59\x59" + payload

            expected_checksum = sum(frame[:8]) & 0xFF

            if expected_checksum != frame[8]:
                continue

            distance_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            temperature_raw = frame[6] | (frame[7] << 8)

            with state.lock:
                state.lidar_distance_m = distance_cm / 100.0
                state.lidar_strength = strength
                state.lidar_temperature_c = (
                    temperature_raw / 8.0 - 256.0
                )
                state.lidar_last_update = time.monotonic()


def format_optional(value, precision: int = 2) -> str:
    if value is None:
        return "N/A"

    if isinstance(value, float):
        return f"{value:.{precision}f}"

    return str(value)


def print_status(state: SharedState) -> None:
    with state.lock:
        print(
            "\n"
            f"Mode: {state.mode:<10} "
            f"Armed: {state.armed}\n"
            f"GPS: "
            f"{format_optional(state.latitude, 7)}, "
            f"{format_optional(state.longitude, 7)}\n"
            f"Heading: "
            f"{format_optional(state.heading_deg, 1)} deg    "
            f"Speed: "
            f"{format_optional(state.ground_speed_mps, 2)} m/s\n"
            f"Battery: "
            f"{format_optional(state.battery_voltage_v, 2)} V, "
            f"{format_optional(state.battery_remaining_pct)}%\n"
            f"LiDAR: "
            f"{format_optional(state.lidar_distance_m, 2)} m, "
            f"strength={format_optional(state.lidar_strength)}, "
            f"temp={format_optional(state.lidar_temperature_c, 1)} C"
        )


def main() -> None:
    state = SharedState()
    stop_event = threading.Event()

    pixhawk_thread = threading.Thread(
        target=read_pixhawk,
        args=(state, stop_event),
        daemon=True,
    )

    lidar_thread = threading.Thread(
        target=read_lidar,
        args=(state, stop_event),
        daemon=True,
    )

    pixhawk_thread.start()
    lidar_thread.start()

    try:
        while True:
            time.sleep(1)
            print_status(state)

    except KeyboardInterrupt:
        print("\nStopping...")
        stop_event.set()

        pixhawk_thread.join(timeout=2)
        lidar_thread.join(timeout=2)

        print("Stopped")


if __name__ == "__main__":
    main()
