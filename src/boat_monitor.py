#!/usr/bin/env python3

import csv
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import serial
from pymavlink import mavutil


PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 115200

LIDAR_PORT = "/dev/ttyAMA3"
LIDAR_BAUD = 115200

PRINT_INTERVAL_S = 1.0
LOG_INTERVAL_S = 0.5

PIXHAWK_STALE_S = 3.0
GPS_STALE_S = 3.0
RC_STALE_S = 3.0
LIDAR_STALE_S = 1.0

PROJECT_DIR = Path.home() / "boat"
DATA_DIR = PROJECT_DIR / "data"
LOG_DIR = PROJECT_DIR / "logs"


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)

    mode: str = "UNKNOWN"
    armed: bool = False

    latitude: Optional[float] = None
    longitude: Optional[float] = None
    heading_deg: Optional[float] = None
    ground_speed_mps: Optional[float] = None

    gps_fix_type: Optional[int] = None
    satellites_visible: Optional[int] = None

    battery_voltage_v: Optional[float] = None
    battery_current_a: Optional[float] = None
    battery_remaining_pct: Optional[int] = None

    rc_rssi: Optional[int] = None
    rc_channel_1: Optional[int] = None
    rc_channel_2: Optional[int] = None
    rc_channel_3: Optional[int] = None
    rc_channel_4: Optional[int] = None

    lidar_distance_m: Optional[float] = None
    lidar_strength: Optional[int] = None
    lidar_temperature_c: Optional[float] = None

    pixhawk_last_update: Optional[float] = None
    gps_last_update: Optional[float] = None
    rc_last_update: Optional[float] = None
    lidar_last_update: Optional[float] = None

    pixhawk_error: Optional[str] = None
    lidar_error: Optional[str] = None


def age_seconds(last_update: Optional[float], now: float) -> Optional[float]:
    if last_update is None:
        return None
    return max(0.0, now - last_update)


def health_status(
    last_update: Optional[float],
    stale_after_s: float,
    now: float,
) -> str:
    age = age_seconds(last_update, now)

    if age is None:
        return "NO_DATA"

    if age > stale_after_s:
        return "STALE"

    return "OK"


def optional_text(value, decimals: int = 2) -> str:
    if value is None:
        return "N/A"

    if isinstance(value, float):
        return f"{value:.{decimals}f}"

    return str(value)


def read_pixhawk(
    state: SharedState,
    stop_event: threading.Event,
) -> None:
    connection = None

    try:
        print(
            f"[Pixhawk] Opening {PIXHAWK_PORT} "
            f"at {PIXHAWK_BAUD} baud"
        )

        connection = mavutil.mavlink_connection(
            PIXHAWK_PORT,
            baud=PIXHAWK_BAUD,
            source_system=255,
        )

        heartbeat = connection.wait_heartbeat(timeout=15)

        print(
            "[Pixhawk] Connected:",
            f"system={connection.target_system}",
            f"component={connection.target_component}",
            f"vehicle_type={heartbeat.type}",
        )

        while not stop_event.is_set():
            message = connection.recv_match(
                blocking=True,
                timeout=1,
            )

            if message is None:
                continue

            message_type = message.get_type()

            if message_type == "BAD_DATA":
                continue

            now = time.monotonic()

            with state.lock:
                state.pixhawk_last_update = now
                state.pixhawk_error = None

                if message_type == "HEARTBEAT":
                    state.mode = mavutil.mode_string_v10(message)
                    state.armed = bool(
                        message.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                elif message_type == "GLOBAL_POSITION_INT":
                    state.latitude = message.lat / 1e7
                    state.longitude = message.lon / 1e7

                    state.heading_deg = (
                        None
                        if message.hdg == 65535
                        else message.hdg / 100.0
                    )

                    state.gps_last_update = now

                elif message_type == "GPS_RAW_INT":
                    state.gps_fix_type = message.fix_type
                    state.satellites_visible = (
                        None
                        if message.satellites_visible == 255
                        else message.satellites_visible
                    )

                    state.gps_last_update = now

                elif message_type == "VFR_HUD":
                    state.ground_speed_mps = message.groundspeed

                elif message_type == "SYS_STATUS":
                    state.battery_voltage_v = (
                        None
                        if message.voltage_battery == 65535
                        else message.voltage_battery / 1000.0
                    )

                    state.battery_current_a = (
                        None
                        if message.current_battery == -1
                        else message.current_battery / 100.0
                    )

                    state.battery_remaining_pct = (
                        None
                        if message.battery_remaining == -1
                        else message.battery_remaining
                    )

                elif message_type == "RC_CHANNELS":
                    state.rc_rssi = (
                        None if message.rssi == 255 else message.rssi
                    )

                    state.rc_channel_1 = message.chan1_raw
                    state.rc_channel_2 = message.chan2_raw
                    state.rc_channel_3 = message.chan3_raw
                    state.rc_channel_4 = message.chan4_raw
                    state.rc_last_update = now

    except Exception as exc:
        with state.lock:
            state.pixhawk_error = (
                f"{type(exc).__name__}: {exc}"
            )

        print(f"[Pixhawk] ERROR: {state.pixhawk_error}")

    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


def read_lidar(
    state: SharedState,
    stop_event: threading.Event,
) -> None:
    try:
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

                checksum = sum(frame[:8]) & 0xFF

                if checksum != frame[8]:
                    continue

                distance_cm = frame[2] | (frame[3] << 8)
                strength = frame[4] | (frame[5] << 8)
                temperature_raw = frame[6] | (frame[7] << 8)

                now = time.monotonic()

                with state.lock:
                    state.lidar_distance_m = distance_cm / 100.0
                    state.lidar_strength = strength
                    state.lidar_temperature_c = (
                        temperature_raw / 8.0 - 256.0
                    )

                    state.lidar_last_update = now
                    state.lidar_error = None

    except Exception as exc:
        with state.lock:
            state.lidar_error = f"{type(exc).__name__}: {exc}"

        print(f"[LiDAR] ERROR: {state.lidar_error}")


def snapshot_state(state: SharedState) -> dict:
    now = time.monotonic()

    with state.lock:
        return {
            "timestamp": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "mode": state.mode,
            "armed": state.armed,
            "latitude": state.latitude,
            "longitude": state.longitude,
            "heading_deg": state.heading_deg,
            "ground_speed_mps": state.ground_speed_mps,
            "gps_fix_type": state.gps_fix_type,
            "satellites_visible": state.satellites_visible,
            "battery_voltage_v": state.battery_voltage_v,
            "battery_current_a": state.battery_current_a,
            "battery_remaining_pct": state.battery_remaining_pct,
            "rc_rssi": state.rc_rssi,
            "rc_channel_1": state.rc_channel_1,
            "rc_channel_2": state.rc_channel_2,
            "rc_channel_3": state.rc_channel_3,
            "rc_channel_4": state.rc_channel_4,
            "lidar_distance_m": state.lidar_distance_m,
            "lidar_strength": state.lidar_strength,
            "lidar_temperature_c": state.lidar_temperature_c,
            "pixhawk_age_s": age_seconds(
                state.pixhawk_last_update,
                now,
            ),
            "gps_age_s": age_seconds(
                state.gps_last_update,
                now,
            ),
            "rc_age_s": age_seconds(
                state.rc_last_update,
                now,
            ),
            "lidar_age_s": age_seconds(
                state.lidar_last_update,
                now,
            ),
            "pixhawk_status": health_status(
                state.pixhawk_last_update,
                PIXHAWK_STALE_S,
                now,
            ),
            "gps_status": health_status(
                state.gps_last_update,
                GPS_STALE_S,
                now,
            ),
            "rc_status": health_status(
                state.rc_last_update,
                RC_STALE_S,
                now,
            ),
            "lidar_status": health_status(
                state.lidar_last_update,
                LIDAR_STALE_S,
                now,
            ),
            "pixhawk_error": state.pixhawk_error,
            "lidar_error": state.lidar_error,
        }


def print_status(snapshot: dict) -> None:
    print(
        "\n"
        f"[{snapshot['timestamp']}]\n"
        f"Pixhawk: {snapshot['pixhawk_status']:<8} "
        f"Mode={snapshot['mode']:<10} "
        f"Armed={snapshot['armed']}\n"
        f"GPS: {snapshot['gps_status']:<8} "
        f"fix={optional_text(snapshot['gps_fix_type'])} "
        f"sats={optional_text(snapshot['satellites_visible'])} "
        f"lat={optional_text(snapshot['latitude'], 7)} "
        f"lon={optional_text(snapshot['longitude'], 7)}\n"
        f"Motion: heading="
        f"{optional_text(snapshot['heading_deg'], 1)} deg "
        f"speed={optional_text(snapshot['ground_speed_mps'], 2)} m/s\n"
        f"RC: {snapshot['rc_status']:<8} "
        f"RSSI={optional_text(snapshot['rc_rssi'])} "
        f"CH1={optional_text(snapshot['rc_channel_1'])} "
        f"CH2={optional_text(snapshot['rc_channel_2'])} "
        f"CH3={optional_text(snapshot['rc_channel_3'])} "
        f"CH4={optional_text(snapshot['rc_channel_4'])}\n"
        f"Battery: "
        f"{optional_text(snapshot['battery_voltage_v'], 2)} V, "
        f"{optional_text(snapshot['battery_current_a'], 2)} A, "
        f"{optional_text(snapshot['battery_remaining_pct'])}%\n"
        f"LiDAR: {snapshot['lidar_status']:<8} "
        f"distance="
        f"{optional_text(snapshot['lidar_distance_m'], 2)} m "
        f"strength="
        f"{optional_text(snapshot['lidar_strength'])} "
        f"temperature="
        f"{optional_text(snapshot['lidar_temperature_c'], 1)} C"
    )


def create_csv_writer():
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = DATA_DIR / f"boat_run_{run_timestamp}.csv"

    csv_file = csv_path.open(
        "w",
        newline="",
        encoding="utf-8",
    )

    fieldnames = list(snapshot_state(SharedState()).keys())

    writer = csv.DictWriter(
        csv_file,
        fieldnames=fieldnames,
    )

    writer.writeheader()
    csv_file.flush()

    return csv_path, csv_file, writer


def main() -> None:
    state = SharedState()
    stop_event = threading.Event()

    csv_path, csv_file, csv_writer = create_csv_writer()

    print(f"[Logger] Writing CSV to {csv_path}")

    pixhawk_thread = threading.Thread(
        target=read_pixhawk,
        args=(state, stop_event),
        daemon=True,
        name="pixhawk-reader",
    )

    lidar_thread = threading.Thread(
        target=read_lidar,
        args=(state, stop_event),
        daemon=True,
        name="lidar-reader",
    )

    pixhawk_thread.start()
    lidar_thread.start()

    last_print = 0.0
    last_log = 0.0

    try:
        while True:
            now = time.monotonic()
            snapshot = snapshot_state(state)

            if now - last_print >= PRINT_INTERVAL_S:
                print_status(snapshot)
                last_print = now

            if now - last_log >= LOG_INTERVAL_S:
                csv_writer.writerow(snapshot)
                csv_file.flush()
                last_log = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
        stop_event.set()

    finally:
        pixhawk_thread.join(timeout=2)
        lidar_thread.join(timeout=2)

        csv_file.flush()
        csv_file.close()

        print(f"[Logger] CSV saved to {csv_path}")
        print("[Main] Stopped")


if __name__ == "__main__":
    main()
