#!/usr/bin/env python3

import time
from pymavlink import mavutil


SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200


def main() -> None:
    print(f"Connecting to Pixhawk on {SERIAL_PORT} at {BAUD_RATE} baud...")

    connection = mavutil.mavlink_connection(
        SERIAL_PORT,
        baud=BAUD_RATE,
        source_system=255,
    )

    heartbeat = connection.wait_heartbeat(timeout=15)

    print(
        "Connected:",
        f"system={connection.target_system}",
        f"component={connection.target_component}",
        f"vehicle_type={heartbeat.type}",
        f"autopilot={heartbeat.autopilot}",
    )

    last_print = {
        "heartbeat": 0.0,
        "position": 0.0,
        "attitude": 0.0,
        "battery": 0.0,
    }

    while True:
        message = connection.recv_match(blocking=True, timeout=2)

        if message is None:
            print("No MAVLink message received for 2 seconds")
            continue

        message_type = message.get_type()
        now = time.monotonic()

        if message_type == "BAD_DATA":
            continue

        if message_type == "HEARTBEAT" and now - last_print["heartbeat"] >= 1:
            mode = mavutil.mode_string_v10(message)
            armed = bool(
                message.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            print(f"[STATE] mode={mode}, armed={armed}")
            last_print["heartbeat"] = now

        elif (
            message_type == "GLOBAL_POSITION_INT"
            and now - last_print["position"] >= 1
        ):
            latitude = message.lat / 1e7
            longitude = message.lon / 1e7
            relative_altitude_m = message.relative_alt / 1000
            heading_deg = (
                None if message.hdg == 65535 else message.hdg / 100
            )

            print(
                "[POSITION]",
                f"lat={latitude:.7f}",
                f"lon={longitude:.7f}",
                f"relative_alt={relative_altitude_m:.2f} m",
                f"heading={heading_deg}",
            )
            last_print["position"] = now

        elif (
            message_type == "ATTITUDE"
            and now - last_print["attitude"] >= 1
        ):
            print(
                "[ATTITUDE]",
                f"roll={message.roll:.3f} rad",
                f"pitch={message.pitch:.3f} rad",
                f"yaw={message.yaw:.3f} rad",
            )
            last_print["attitude"] = now

        elif (
            message_type == "SYS_STATUS"
            and now - last_print["battery"] >= 2
        ):
            voltage_v = (
                None
                if message.voltage_battery == 65535
                else message.voltage_battery / 1000
            )

            current_a = (
                None
                if message.current_battery == -1
                else message.current_battery / 100
            )

            print(
                "[BATTERY]",
                f"voltage={voltage_v} V",
                f"current={current_a} A",
                f"remaining={message.battery_remaining}%",
            )
            last_print["battery"] = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
    except TimeoutError:
        print("Timed out waiting for Pixhawk heartbeat")
    except PermissionError as exc:
        print(f"Serial permission error: {exc}")
    except Exception as exc:
        print(f"Unexpected error: {type(exc).__name__}: {exc}")
