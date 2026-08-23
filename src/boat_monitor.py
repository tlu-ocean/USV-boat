#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import serial
from pyais import decode
from pymavlink import mavutil

import board
import adafruit_bme280.basic as adafruit_bme280


# ============================================================
# Configuration
# ============================================================

ENABLE_PIXHAWK = True
ENABLE_LIDAR = True
ENABLE_BME280 = True
ENABLE_DS18B20 = True
ENABLE_AIS = True


# ---------------- Pixhawk ----------------

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 115200
PIXHAWK_STALE_S = 3.0


# ---------------- TFmini Plus ----------------

LIDAR_PORT = "/dev/ttyAMA3"
LIDAR_BAUD = 115200
LIDAR_STALE_S = 2.0


# ---------------- BME280 ----------------

BME280_ADDRESS = 0x76
BME280_READ_INTERVAL_S = 1.0
BME280_STALE_S = 3.0


# ---------------- DS18B20 ----------------

DS18B20_DEVICE_ID = "28-000000230f20"
DS18B20_BASE_DIR = Path("/sys/bus/w1/devices")
DS18B20_READ_INTERVAL_S = 1.0
DS18B20_STALE_S = 3.0


# ---------------- AIS / AR-10 ----------------

# Stable path for the current CH340/CH341-based AR-10 receiver.
AIS_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
AIS_BAUD = 38400
AIS_STALE_S = 10.0
AIS_RECONNECT_DELAY_S = 5.0
AIS_TARGET_ACTIVE_S = 60.0
AIS_VESSEL_REMOVE_S = 300.0
AIS_TABLE_MAX_ROWS = 12

# If Pixhawk has no valid GPS fix, use this point only as a temporary
# reference for distance/bearing calculations. It does NOT overwrite
# Pixhawk GPS data and is always labeled FALLBACK in the display/log.
USE_FALLBACK_POSITION = True
FALLBACK_LAT = 29.313499
FALLBACK_LON = -94.817141


# ---------------- General ----------------

RECONNECT_DELAY_S = 5.0
MAIN_LOOP_INTERVAL_S = 1.0
DATA_DIR = Path.home() / "boat" / "data"
AIS_DATA_DIR = DATA_DIR / "ais"


# ============================================================
# Shared state
# ============================================================

@dataclass
class SharedState:

    # --------------------------------------------------------
    # System
    # --------------------------------------------------------

    timestamp: str | None = None

    # --------------------------------------------------------
    # Pixhawk
    # --------------------------------------------------------

    pixhawk_status: str = "NO_DATA"
    pixhawk_error: str | None = None
    pixhawk_last_update: float | None = None

    flight_mode: str | None = None
    armed: bool | None = None

    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_m: float | None = None

    gps_fix_type: int | None = None
    gps_satellites: int | None = None

    heading_deg: float | None = None
    ground_course_deg: float | None = None
    ground_speed_mps: float | None = None

    battery_voltage_v: float | None = None
    battery_current_a: float | None = None
    battery_remaining_pct: int | None = None

    rc1: int | None = None
    rc2: int | None = None
    rc3: int | None = None
    rc4: int | None = None

    # --------------------------------------------------------
    # TFmini Plus
    # --------------------------------------------------------

    lidar_status: str = "NO_DATA"
    lidar_error: str | None = None
    lidar_last_update: float | None = None

    lidar_distance_m: float | None = None
    lidar_strength: int | None = None
    lidar_temperature_c: float | None = None

    # --------------------------------------------------------
    # BME280
    # --------------------------------------------------------

    bme280_status: str = "NO_DATA"
    bme280_error: str | None = None
    bme280_last_update: float | None = None

    air_temperature_c: float | None = None
    relative_humidity_pct: float | None = None
    air_pressure_hpa: float | None = None

    # --------------------------------------------------------
    # DS18B20
    # --------------------------------------------------------

    ds18b20_status: str = "NO_DATA"
    ds18b20_error: str | None = None
    ds18b20_last_update: float | None = None

    ds18b20_temperature_c: float | None = None

    # --------------------------------------------------------
    # AIS summary
    # --------------------------------------------------------

    ais_status: str = "NO_DATA"
    ais_error: str | None = None
    ais_last_update: float | None = None
    ais_vessel_count: int = 0


state = SharedState()
state_lock = threading.Lock()

# One entry per MMSI. Dynamic messages update the same row; static
# messages fill in name/callsign/type/etc for that same MMSI.
ais_vessels: dict[int, dict] = {}
ais_lock = threading.Lock()

# Multipart !AIVDM cache.
ais_multipart: dict[tuple, dict] = {}


# ============================================================
# Utility functions
# ============================================================

def now_monotonic() -> float:
    return time.monotonic()


def wall_clock_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def age_seconds(last_update: float | None) -> float | None:
    if last_update is None:
        return None
    return now_monotonic() - last_update


def status_from_age(
    enabled: bool,
    last_update: float | None,
    stale_threshold_s: float,
) -> str:
    if not enabled:
        return "DISABLED"
    if last_update is None:
        return "NO_DATA"

    age = age_seconds(last_update)
    if age is not None and age > stale_threshold_s:
        return "STALE"

    return "OK"


def fmt(value, decimals=2):
    if value is None:
        return "--"
    if isinstance(value, float):
        return f"{value:.{decimals}f}"
    return str(value)


def valid_pixhawk_position(
    lat: float | None,
    lon: float | None,
    fix_type: int | None,
) -> bool:
    if fix_type is None or fix_type < 2:
        return False
    if lat is None or lon is None:
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        return False
    return True


def get_own_position(snapshot: dict) -> tuple[float | None, float | None, str]:
    lat = snapshot.get("latitude_deg")
    lon = snapshot.get("longitude_deg")
    fix_type = snapshot.get("gps_fix_type")

    if valid_pixhawk_position(lat, lon, fix_type):
        return lat, lon, "PIXHAWK"

    if USE_FALLBACK_POSITION:
        return FALLBACK_LAT, FALLBACK_LON, "FALLBACK"

    return None, None, "UNAVAILABLE"


def distance_and_bearing(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
) -> tuple[float, float]:
    """Return great-circle distance in meters and initial bearing in degrees."""

    earth_radius_m = 6371000.0

    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    distance_m = earth_radius_m * c

    y = math.sin(dlon) * math.cos(lat2)
    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )
    bearing_deg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    return distance_m, bearing_deg


def signed_angle_deg(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def relative_bearing_deg(
    target_bearing_deg: float | None,
    own_heading_deg: float | None,
) -> float | None:
    """
    Relative bearing from own bow to the target.

    0 = straight ahead
    +90 = starboard/right
    -90 = port/left
    +/-180 = directly astern
    """
    if target_bearing_deg is None or own_heading_deg is None:
        return None
    return signed_angle_deg(target_bearing_deg - own_heading_deg)


def velocity_components_mps(
    speed_mps: float,
    course_deg: float,
) -> tuple[float, float]:
    """Return east, north velocity components from SOG/course."""
    angle = math.radians(course_deg)
    east = speed_mps * math.sin(angle)
    north = speed_mps * math.cos(angle)
    return east, north


def cpa_tcpa(
    distance_m: float | None,
    bearing_deg: float | None,
    own_speed_mps: float | None,
    own_course_deg: float | None,
    target_speed_knots: float | None,
    target_course_deg: float | None,
) -> tuple[float | None, float | None]:
    """
    Constant-velocity Closest Point of Approach calculation.

    Returns:
        cpa_m: separation distance at closest approach
        tcpa_s: signed time to CPA in seconds
                > 0 means closest approach is in the future
                < 0 means the mathematical CPA was in the past
    """
    values = (
        distance_m,
        bearing_deg,
        own_speed_mps,
        own_course_deg,
        target_speed_knots,
        target_course_deg,
    )
    if any(value is None for value in values):
        return None, None

    bearing = math.radians(float(bearing_deg))
    rel_east = float(distance_m) * math.sin(bearing)
    rel_north = float(distance_m) * math.cos(bearing)

    own_east, own_north = velocity_components_mps(
        float(own_speed_mps),
        float(own_course_deg),
    )
    target_east, target_north = velocity_components_mps(
        float(target_speed_knots) * 0.514444,
        float(target_course_deg),
    )

    rel_vel_east = target_east - own_east
    rel_vel_north = target_north - own_north
    rel_speed_sq = rel_vel_east ** 2 + rel_vel_north ** 2

    # If relative speed is essentially zero, CPA is simply current range
    # and there is no meaningful finite TCPA.
    if rel_speed_sq < 1e-6:
        return float(distance_m), None

    tcpa_s = -(
        rel_east * rel_vel_east
        + rel_north * rel_vel_north
    ) / rel_speed_sq

    cpa_east = rel_east + rel_vel_east * tcpa_s
    cpa_north = rel_north + rel_vel_north * tcpa_s
    cpa_m = math.hypot(cpa_east, cpa_north)

    return cpa_m, tcpa_s


# ============================================================
# Pixhawk reader
# ============================================================

def read_pixhawk() -> None:
    if not ENABLE_PIXHAWK:
        with state_lock:
            state.pixhawk_status = "DISABLED"
        return

    while True:
        connection = None

        try:
            print(f"[Pixhawk] Connecting to {PIXHAWK_PORT} @ {PIXHAWK_BAUD}")

            connection = mavutil.mavlink_connection(
                PIXHAWK_PORT,
                baud=PIXHAWK_BAUD,
            )

            connection.wait_heartbeat(timeout=10)

            print(
                "[Pixhawk] Heartbeat received "
                f"system={connection.target_system} "
                f"component={connection.target_component}"
            )

            with state_lock:
                state.pixhawk_error = None

            while True:
                msg = connection.recv_match(
                    blocking=True,
                    timeout=1,
                )

                if msg is None:
                    continue

                msg_type = msg.get_type()
                now = now_monotonic()

                if msg_type == "HEARTBEAT":
                    try:
                        mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        mode = None

                    armed = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                    with state_lock:
                        state.flight_mode = mode
                        state.armed = armed
                        state.pixhawk_last_update = now
                        state.pixhawk_error = None

                elif msg_type == "GPS_RAW_INT":
                    latitude = msg.lat / 1e7
                    longitude = msg.lon / 1e7
                    altitude_m = msg.alt / 1000.0

                    with state_lock:
                        state.latitude_deg = latitude
                        state.longitude_deg = longitude
                        state.altitude_m = altitude_m
                        state.gps_fix_type = msg.fix_type
                        state.gps_satellites = msg.satellites_visible
                        state.pixhawk_last_update = now
                        state.pixhawk_error = None

                elif msg_type == "GLOBAL_POSITION_INT":
                    heading_deg = None

                    if msg.hdg != 65535:
                        heading_deg = msg.hdg / 100.0

                    ground_speed_mps = (
                        (msg.vx ** 2 + msg.vy ** 2) ** 0.5
                        / 100.0
                    )

                    ground_course_deg = None
                    if ground_speed_mps > 0.05:
                        # MAVLink GLOBAL_POSITION_INT uses vx=north, vy=east.
                        ground_course_deg = (
                            math.degrees(math.atan2(msg.vy, msg.vx)) + 360.0
                        ) % 360.0

                    with state_lock:
                        # Do not let GLOBAL_POSITION_INT replace a known invalid
                        # GPS_RAW_INT fix with an apparently valid-looking position
                        # for own-ship selection; get_own_position() still requires
                        # gps_fix_type >= 2.
                        state.latitude_deg = msg.lat / 1e7
                        state.longitude_deg = msg.lon / 1e7
                        state.heading_deg = heading_deg
                        state.ground_course_deg = ground_course_deg
                        state.ground_speed_mps = ground_speed_mps
                        state.pixhawk_last_update = now
                        state.pixhawk_error = None

                elif msg_type == "VFR_HUD":
                    with state_lock:
                        state.heading_deg = float(msg.heading)
                        state.ground_speed_mps = float(msg.groundspeed)
                        state.pixhawk_last_update = now
                        state.pixhawk_error = None

                elif msg_type == "SYS_STATUS":
                    voltage_v = None
                    current_a = None
                    remaining = None

                    if msg.voltage_battery != 65535:
                        voltage_v = msg.voltage_battery / 1000.0

                    if msg.current_battery != -1:
                        current_a = msg.current_battery / 100.0

                    if msg.battery_remaining != -1:
                        remaining = int(msg.battery_remaining)

                    with state_lock:
                        state.battery_voltage_v = voltage_v
                        state.battery_current_a = current_a
                        state.battery_remaining_pct = remaining
                        state.pixhawk_last_update = now
                        state.pixhawk_error = None

                elif msg_type == "RC_CHANNELS":
                    with state_lock:
                        state.rc1 = msg.chan1_raw
                        state.rc2 = msg.chan2_raw
                        state.rc3 = msg.chan3_raw
                        state.rc4 = msg.chan4_raw
                        state.pixhawk_last_update = now
                        state.pixhawk_error = None

        except Exception as exc:
            print(f"[Pixhawk] ERROR: {exc}")
            with state_lock:
                state.pixhawk_error = str(exc)

        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass

        print(f"[Pixhawk] Reconnecting in {RECONNECT_DELAY_S:.0f} s...")
        time.sleep(RECONNECT_DELAY_S)


# ============================================================
# TFmini Plus reader
# ============================================================

def parse_tfmini_frame(frame: bytes):
    if len(frame) != 9:
        return None
    if frame[0] != 0x59 or frame[1] != 0x59:
        return None

    checksum = sum(frame[:8]) & 0xFF
    if checksum != frame[8]:
        return None

    distance_cm = frame[2] | (frame[3] << 8)
    strength = frame[4] | (frame[5] << 8)
    temperature_raw = frame[6] | (frame[7] << 8)
    temperature_c = temperature_raw / 8.0 - 256.0

    return (
        distance_cm / 100.0,
        strength,
        temperature_c,
    )


def read_lidar() -> None:
    if not ENABLE_LIDAR:
        with state_lock:
            state.lidar_status = "DISABLED"
        return

    while True:
        ser = None

        try:
            print(f"[LiDAR] Opening {LIDAR_PORT} @ {LIDAR_BAUD}")

            ser = serial.Serial(
                LIDAR_PORT,
                LIDAR_BAUD,
                timeout=1,
            )

            ser.reset_input_buffer()
            print("[LiDAR] Connected")

            with state_lock:
                state.lidar_error = None

            buffer = bytearray()

            while True:
                chunk = ser.read(64)
                if not chunk:
                    continue

                buffer.extend(chunk)

                while len(buffer) >= 9:
                    header_index = buffer.find(b"\x59\x59")

                    if header_index < 0:
                        if buffer[-1:] == b"\x59":
                            buffer[:] = buffer[-1:]
                        else:
                            buffer.clear()
                        break

                    if header_index > 0:
                        del buffer[:header_index]

                    if len(buffer) < 9:
                        break

                    frame = bytes(buffer[:9])
                    parsed = parse_tfmini_frame(frame)

                    if parsed is None:
                        del buffer[0]
                        continue

                    del buffer[:9]

                    distance_m, strength, temperature_c = parsed

                    with state_lock:
                        state.lidar_distance_m = distance_m
                        state.lidar_strength = strength
                        state.lidar_temperature_c = temperature_c
                        state.lidar_last_update = now_monotonic()
                        state.lidar_error = None

        except Exception as exc:
            print(f"[LiDAR] ERROR: {exc}")
            with state_lock:
                state.lidar_error = str(exc)

        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

        print(f"[LiDAR] Reconnecting in {RECONNECT_DELAY_S:.0f} s...")
        time.sleep(RECONNECT_DELAY_S)


# ============================================================
# BME280 reader
# ============================================================

def read_bme280() -> None:
    if not ENABLE_BME280:
        with state_lock:
            state.bme280_status = "DISABLED"
        return

    while True:
        try:
            print(
                f"[BME280] Connecting on I2C "
                f"address 0x{BME280_ADDRESS:02X}"
            )

            i2c = board.I2C()

            sensor = adafruit_bme280.Adafruit_BME280_I2C(
                i2c,
                address=BME280_ADDRESS,
            )

            print("[BME280] Connected")

            with state_lock:
                state.bme280_error = None

            while True:
                temperature_c = float(sensor.temperature)
                humidity_pct = float(sensor.relative_humidity)
                pressure_hpa = float(sensor.pressure)

                with state_lock:
                    state.air_temperature_c = temperature_c
                    state.relative_humidity_pct = humidity_pct
                    state.air_pressure_hpa = pressure_hpa
                    state.bme280_last_update = now_monotonic()
                    state.bme280_error = None

                time.sleep(BME280_READ_INTERVAL_S)

        except Exception as exc:
            print(f"[BME280] ERROR: {exc}")
            with state_lock:
                state.bme280_error = str(exc)

        print(f"[BME280] Reconnecting in {RECONNECT_DELAY_S:.0f} s...")
        time.sleep(RECONNECT_DELAY_S)


# ============================================================
# DS18B20 reader
# ============================================================

def get_ds18b20_file() -> Path:
    return DS18B20_BASE_DIR / DS18B20_DEVICE_ID / "w1_slave"


def read_ds18b20_temperature() -> float:
    sensor_file = get_ds18b20_file()
    text = sensor_file.read_text()
    lines = text.strip().splitlines()

    if len(lines) < 2:
        raise RuntimeError("Unexpected DS18B20 data format")

    if not lines[0].strip().endswith("YES"):
        raise RuntimeError("DS18B20 CRC check failed")

    marker = "t="
    position = lines[1].find(marker)

    if position < 0:
        raise RuntimeError("DS18B20 temperature field not found")

    temperature_mdeg = int(lines[1][position + len(marker):])
    return temperature_mdeg / 1000.0


def read_ds18b20() -> None:
    if not ENABLE_DS18B20:
        with state_lock:
            state.ds18b20_status = "DISABLED"
        return

    while True:
        try:
            sensor_file = get_ds18b20_file()
            print(f"[DS18B20] Looking for {sensor_file}")

            if not sensor_file.exists():
                raise FileNotFoundError(
                    f"DS18B20 not found: {DS18B20_DEVICE_ID}"
                )

            print(f"[DS18B20] Connected {DS18B20_DEVICE_ID}")

            with state_lock:
                state.ds18b20_error = None

            while True:
                temperature_c = read_ds18b20_temperature()

                with state_lock:
                    state.ds18b20_temperature_c = temperature_c
                    state.ds18b20_last_update = now_monotonic()
                    state.ds18b20_error = None

                time.sleep(DS18B20_READ_INTERVAL_S)

        except Exception as exc:
            print(f"[DS18B20] ERROR: {exc}")
            with state_lock:
                state.ds18b20_error = str(exc)

        print(f"[DS18B20] Reconnecting in {RECONNECT_DELAY_S:.0f} s...")
        time.sleep(RECONNECT_DELAY_S)


# ============================================================
# AIS helpers
# ============================================================

def clean_ais_text(value):
    if value is None:
        return None
    text = str(value).replace("@", "").strip()
    return text or None


def clean_ais_heading(value):
    # AIS 511 means heading unavailable.
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if 0.0 <= value <= 359.0 else None


def clean_ais_course(value):
    # AIS 360.0 means COG unavailable.
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if 0.0 <= value < 360.0 else None


def clean_ais_speed(value):
    # AIS 102.3 kn means SOG unavailable.
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if 0.0 <= value < 102.3 else None


def clean_ais_position(lat, lon):
    if lat is None or lon is None:
        return None, None

    try:
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        return None, None

    if not (-90.0 <= lat <= 90.0):
        return None, None
    if not (-180.0 <= lon <= 180.0):
        return None, None

    return lat, lon


def new_ais_vessel(mmsi: int) -> dict:
    return {
        "mmsi": mmsi,
        "lat": None,
        "lon": None,
        "speed_knots": None,
        "course_deg": None,
        "heading_deg": None,
        "rot": None,
        "nav_status": None,
        "name": None,
        "callsign": None,
        "imo": None,
        "ship_type": None,
        "destination": None,
        "draught_m": None,
        "to_bow_m": None,
        "to_stern_m": None,
        "to_port_m": None,
        "to_starboard_m": None,
        "last_msg_type": None,
        "last_seen_monotonic": None,
        "last_position_monotonic": None,
    }


def ais_history_file() -> Path:
    AIS_DATA_DIR.mkdir(parents=True, exist_ok=True)
    return AIS_DATA_DIR / f"ais_{datetime.now().strftime('%Y%m%d')}.csv"


def log_ais_position(msg_type: int, vessel: dict) -> None:
    path = ais_history_file()
    file_exists = path.exists()

    with path.open("a", newline="", buffering=1) as f:
        writer = csv.writer(f)

        if not file_exists:
            writer.writerow([
                "timestamp_utc",
                "msg_type",
                "mmsi",
                "lat",
                "lon",
                "speed_knots",
                "course_deg",
                "heading_deg",
            ])

        writer.writerow([
            datetime.now(timezone.utc).isoformat(timespec="seconds"),
            msg_type,
            vessel["mmsi"],
            vessel["lat"],
            vessel["lon"],
            vessel["speed_knots"],
            vessel["course_deg"],
            vessel["heading_deg"],
        ])


def process_ais_message(msg) -> None:
    data = msg.asdict()
    msg_type = data.get("msg_type")
    mmsi = data.get("mmsi")

    if mmsi is None:
        return

    try:
        mmsi = int(mmsi)
    except (TypeError, ValueError):
        return

    now = now_monotonic()

    with ais_lock:
        vessel = ais_vessels.setdefault(mmsi, new_ais_vessel(mmsi))
        vessel["last_seen_monotonic"] = now
        vessel["last_msg_type"] = msg_type

        # Class A dynamic: 1/2/3
        # Class B dynamic: 18/19
        if msg_type in {1, 2, 3, 18, 19}:
            lat, lon = clean_ais_position(
                data.get("lat"),
                data.get("lon"),
            )

            if lat is not None and lon is not None:
                vessel["lat"] = lat
                vessel["lon"] = lon
                vessel["last_position_monotonic"] = now

            vessel["speed_knots"] = clean_ais_speed(data.get("speed"))
            vessel["course_deg"] = clean_ais_course(data.get("course"))
            vessel["heading_deg"] = clean_ais_heading(data.get("heading"))
            vessel["rot"] = data.get("turn")
            vessel["nav_status"] = data.get("status")

            if vessel["lat"] is not None and vessel["lon"] is not None:
                log_ais_position(msg_type, vessel.copy())

        # Class A static/voyage
        elif msg_type == 5:
            vessel["name"] = clean_ais_text(data.get("shipname"))
            vessel["callsign"] = clean_ais_text(data.get("callsign"))
            vessel["imo"] = data.get("imo")
            vessel["ship_type"] = data.get("ship_type")
            vessel["destination"] = clean_ais_text(data.get("destination"))
            vessel["draught_m"] = data.get("draught")
            vessel["to_bow_m"] = data.get("to_bow")
            vessel["to_stern_m"] = data.get("to_stern")
            vessel["to_port_m"] = data.get("to_port")
            vessel["to_starboard_m"] = data.get("to_starboard")

        # Class B static (part A / part B)
        elif msg_type == 24:
            name = clean_ais_text(data.get("shipname"))
            callsign = clean_ais_text(data.get("callsign"))

            if name:
                vessel["name"] = name
            if callsign:
                vessel["callsign"] = callsign
            if data.get("ship_type") is not None:
                vessel["ship_type"] = data.get("ship_type")

            field_map = {
                "to_bow": "to_bow_m",
                "to_stern": "to_stern_m",
                "to_port": "to_port_m",
                "to_starboard": "to_starboard_m",
            }

            for source, target in field_map.items():
                if data.get(source) is not None:
                    vessel[target] = data.get(source)

    with state_lock:
        state.ais_last_update = now
        state.ais_error = None
        state.ais_vessel_count = len(ais_vessels)


def cleanup_ais_multipart() -> None:
    now = now_monotonic()

    expired = [
        key
        for key, entry in ais_multipart.items()
        if now - entry["created"] > 10.0
    ]

    for key in expired:
        del ais_multipart[key]


def decode_ais_line(line: str) -> None:
    parts = line.split(",")

    if len(parts) < 7:
        return

    try:
        total_fragments = int(parts[1])
        fragment_number = int(parts[2])
    except ValueError:
        return

    if total_fragments == 1:
        process_ais_message(decode(line))
        return

    sequence_id = parts[3]
    channel = parts[4]

    # Sequence IDs can be blank. For a single receiver, channel + fragment
    # count + sequence ID is adequate for the normal Type-5/Type-24 traffic
    # expected here. Old incomplete groups are purged after 10 seconds.
    key = (sequence_id, channel, total_fragments)

    entry = ais_multipart.setdefault(
        key,
        {
            "created": now_monotonic(),
            "fragments": {},
        },
    )

    entry["fragments"][fragment_number] = line

    if len(entry["fragments"]) == total_fragments:
        fragments = [
            entry["fragments"][i]
            for i in range(1, total_fragments + 1)
        ]

        del ais_multipart[key]
        process_ais_message(decode(*fragments))

    cleanup_ais_multipart()


def cleanup_old_ais_vessels() -> None:
    now = now_monotonic()

    with ais_lock:
        expired = []

        for mmsi, vessel in ais_vessels.items():
            last_seen = vessel.get("last_seen_monotonic")
            if last_seen is not None and now - last_seen > AIS_VESSEL_REMOVE_S:
                expired.append(mmsi)

        for mmsi in expired:
            del ais_vessels[mmsi]

        vessel_count = len(ais_vessels)

    with state_lock:
        state.ais_vessel_count = vessel_count


def read_ais() -> None:
    if not ENABLE_AIS:
        with state_lock:
            state.ais_status = "DISABLED"
        return

    while True:
        ser = None

        try:
            print(f"[AIS] Opening {AIS_PORT} @ {AIS_BAUD}")

            ser = serial.Serial(
                AIS_PORT,
                AIS_BAUD,
                timeout=1,
            )

            ser.reset_input_buffer()
            print("[AIS] Connected")

            with state_lock:
                state.ais_error = None

            while True:
                raw = ser.readline()

                if not raw:
                    cleanup_old_ais_vessels()
                    continue

                line = raw.decode("ascii", errors="ignore").strip()

                if not (
                    line.startswith("!AIVDM")
                    or line.startswith("!AIVDO")
                ):
                    continue

                try:
                    decode_ais_line(line)
                except Exception as exc:
                    # One malformed AIS sentence must not kill the receiver.
                    print(f"[AIS] Decode error: {exc}")

                cleanup_old_ais_vessels()

        except Exception as exc:
            print(f"[AIS] ERROR: {exc}")

            with state_lock:
                state.ais_error = str(exc)

        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

        print(f"[AIS] Reconnecting in {AIS_RECONNECT_DELAY_S:.0f} s...")
        time.sleep(AIS_RECONNECT_DELAY_S)


# ============================================================
# Snapshot
# ============================================================

def snapshot_state() -> dict:
    with state_lock:
        state.pixhawk_status = status_from_age(
            ENABLE_PIXHAWK,
            state.pixhawk_last_update,
            PIXHAWK_STALE_S,
        )

        state.lidar_status = status_from_age(
            ENABLE_LIDAR,
            state.lidar_last_update,
            LIDAR_STALE_S,
        )

        state.bme280_status = status_from_age(
            ENABLE_BME280,
            state.bme280_last_update,
            BME280_STALE_S,
        )

        state.ds18b20_status = status_from_age(
            ENABLE_DS18B20,
            state.ds18b20_last_update,
            DS18B20_STALE_S,
        )

        state.ais_status = status_from_age(
            ENABLE_AIS,
            state.ais_last_update,
            AIS_STALE_S,
        )

        state.timestamp = wall_clock_timestamp()
        snapshot = asdict(state)

    snapshot["pixhawk_age_s"] = age_seconds(
        snapshot.pop("pixhawk_last_update")
    )
    snapshot["lidar_age_s"] = age_seconds(
        snapshot.pop("lidar_last_update")
    )
    snapshot["bme280_age_s"] = age_seconds(
        snapshot.pop("bme280_last_update")
    )
    snapshot["ds18b20_age_s"] = age_seconds(
        snapshot.pop("ds18b20_last_update")
    )
    snapshot["ais_age_s"] = age_seconds(
        snapshot.pop("ais_last_update")
    )

    own_lat, own_lon, own_source = get_own_position(snapshot)
    snapshot["own_position_lat_deg"] = own_lat
    snapshot["own_position_lon_deg"] = own_lon
    snapshot["own_position_source"] = own_source

    return snapshot


def snapshot_ais_vessels() -> list[dict]:
    with ais_lock:
        return [vessel.copy() for vessel in ais_vessels.values()]


# ============================================================
# Terminal display
# ============================================================

def print_ais_table(snapshot: dict) -> None:
    own_lat = snapshot.get("own_position_lat_deg")
    own_lon = snapshot.get("own_position_lon_deg")
    own_source = snapshot.get("own_position_source")
    own_heading = snapshot.get("heading_deg")
    own_course = snapshot.get("ground_course_deg")
    own_speed = snapshot.get("ground_speed_mps")

    vessels = snapshot_ais_vessels()
    now = now_monotonic()

    rows = []

    for vessel in vessels:
        lat = vessel.get("lat")
        lon = vessel.get("lon")

        if lat is None or lon is None:
            continue

        last_position = vessel.get("last_position_monotonic")
        age = None if last_position is None else now - last_position

        target_state = (
            "ACTIVE"
            if age is not None and age <= AIS_TARGET_ACTIVE_S
            else "STALE"
        )

        distance_m = None
        bearing_deg = None
        rel_bearing = None
        cpa_m = None
        tcpa_s = None

        if own_lat is not None and own_lon is not None:
            distance_m, bearing_deg = distance_and_bearing(
                own_lat,
                own_lon,
                lat,
                lon,
            )

            rel_bearing = relative_bearing_deg(
                bearing_deg,
                own_heading,
            )

        # CPA/TCPA needs a real own-ship position and motion solution.
        # Do not produce collision-prediction numbers from the fixed fallback
        # reference point.
        if own_source == "PIXHAWK":
            cpa_m, tcpa_s = cpa_tcpa(
                distance_m,
                bearing_deg,
                own_speed,
                own_course,
                vessel.get("speed_knots"),
                vessel.get("course_deg"),
            )

        rows.append((
            float("inf") if distance_m is None else distance_m,
            vessel,
            age,
            target_state,
            distance_m,
            bearing_deg,
            rel_bearing,
            cpa_m,
            tcpa_s,
        ))

    rows.sort(key=lambda row: row[0])

    print(
        f"AIS       [{snapshot['ais_status']}]  "
        f"vessels={snapshot['ais_vessel_count']}  "
        f"last_msg={fmt(snapshot['ais_age_s'], 1)} s"
    )

    print(
        f"OWN POS   "
        f"lat={fmt(own_lat, 6)}  "
        f"lon={fmt(own_lon, 6)}  "
        f"source={own_source}"
    )

    if not rows:
        print("AIS TARGETS: no vessels with valid position yet")
        return

    print("AIS TARGETS:")
    print(
        "  "
        f"{'MMSI':<10} "
        f"{'NAME':<18} "
        f"{'STATE':<6} "
        f"{'DIST':>8} "
        f"{'BRG':>5} "
        f"{'REL':>6} "
        f"{'SOG':>5} "
        f"{'COG':>6} "
        f"{'HDG':>5} "
        f"{'CPA':>8} "
        f"{'TCPA':>7} "
        f"{'POSAGE':>7}"
    )

    for row in rows[:AIS_TABLE_MAX_ROWS]:
        (
            _, vessel, age, target_state, distance_m, bearing_deg,
            rel_bearing, cpa_m, tcpa_s
        ) = row

        name = (vessel.get("name") or "-")[:18]

        if distance_m is None:
            distance_text = "--"
        elif distance_m < 1000:
            distance_text = f"{distance_m:.0f}m"
        else:
            distance_text = f"{distance_m / 1000.0:.2f}km"

        if cpa_m is None:
            cpa_text = "--"
        elif cpa_m < 1000:
            cpa_text = f"{cpa_m:.0f}m"
        else:
            cpa_text = f"{cpa_m / 1000.0:.2f}km"

        if tcpa_s is None:
            tcpa_text = "--"
        else:
            tcpa_text = f"{tcpa_s / 60.0:.1f}m"

        age_text = "--" if age is None else f"{age:.0f}s"

        print(
            "  "
            f"{vessel['mmsi']:<10} "
            f"{name:<18} "
            f"{target_state:<6} "
            f"{distance_text:>8} "
            f"{fmt(bearing_deg, 0):>5} "
            f"{fmt(rel_bearing, 0):>6} "
            f"{fmt(vessel.get('speed_knots'), 1):>5} "
            f"{fmt(vessel.get('course_deg'), 1):>6} "
            f"{fmt(vessel.get('heading_deg'), 0):>5} "
            f"{cpa_text:>8} "
            f"{tcpa_text:>7} "
            f"{age_text:>7}"
        )


def print_snapshot(data: dict) -> None:
    print()
    print("=" * 100)
    print(data["timestamp"])

    print(
        f"PIXHAWK  [{data['pixhawk_status']}]  "
        f"mode={data['flight_mode']}  "
        f"armed={data['armed']}"
    )

    print(
        f"GPS       "
        f"lat={fmt(data['latitude_deg'], 6)}  "
        f"lon={fmt(data['longitude_deg'], 6)}  "
        f"fix={fmt(data['gps_fix_type'])}  "
        f"sats={fmt(data['gps_satellites'])}"
    )

    print(
        f"NAV       "
        f"heading={fmt(data['heading_deg'], 1)} deg  "
        f"course={fmt(data['ground_course_deg'], 1)} deg  "
        f"speed={fmt(data['ground_speed_mps'], 2)} m/s"
    )

    print(
        f"BATTERY   "
        f"V={fmt(data['battery_voltage_v'], 2)}  "
        f"I={fmt(data['battery_current_a'], 2)}  "
        f"remaining={fmt(data['battery_remaining_pct'])}%"
    )

    print(
        f"RC        "
        f"{fmt(data['rc1'])} "
        f"{fmt(data['rc2'])} "
        f"{fmt(data['rc3'])} "
        f"{fmt(data['rc4'])}"
    )

    print(
        f"LIDAR     [{data['lidar_status']}]  "
        f"d={fmt(data['lidar_distance_m'], 2)} m  "
        f"strength={fmt(data['lidar_strength'])}  "
        f"T_internal={fmt(data['lidar_temperature_c'], 1)} C"
    )

    print(
        f"BME280    [{data['bme280_status']}]  "
        f"T={fmt(data['air_temperature_c'], 2)} C  "
        f"RH={fmt(data['relative_humidity_pct'], 1)}%  "
        f"P={fmt(data['air_pressure_hpa'], 1)} hPa"
    )

    print(
        f"DS18B20   [{data['ds18b20_status']}]  "
        f"T={fmt(data['ds18b20_temperature_c'], 2)} C"
    )

    print_ais_table(data)

    for sensor in (
        "pixhawk",
        "lidar",
        "bme280",
        "ds18b20",
        "ais",
    ):
        error = data.get(f"{sensor}_error")
        if error:
            print(f"{sensor.upper()} ERROR: {error}")


# ============================================================
# CSV
# ============================================================

def create_csv_file(snapshot: dict):
    DATA_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    filename = (
        "boat_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
        + ".csv"
    )

    path = DATA_DIR / filename

    file_handle = path.open(
        "w",
        newline="",
        buffering=1,
    )

    writer = csv.DictWriter(
        file_handle,
        fieldnames=list(snapshot.keys()),
    )

    writer.writeheader()
    print(f"[CSV] Logging to {path}")

    return file_handle, writer


# ============================================================
# Main
# ============================================================

def main() -> None:
    print()
    print("Boat Monitor")
    print("=" * 100)

    threads = []

    readers = [
        ("pixhawk", read_pixhawk),
        ("lidar", read_lidar),
        ("bme280", read_bme280),
        ("ds18b20", read_ds18b20),
        ("ais", read_ais),
    ]

    for name, target in readers:
        thread = threading.Thread(
            target=target,
            name=name,
            daemon=True,
        )

        thread.start()
        threads.append(thread)

    # Let the readers initialize before creating the first CSV snapshot.
    time.sleep(1.0)

    first_snapshot = snapshot_state()
    csv_file, csv_writer = create_csv_file(first_snapshot)

    try:
        while True:
            snapshot = snapshot_state()
            print_snapshot(snapshot)
            csv_writer.writerow(snapshot)
            time.sleep(MAIN_LOOP_INTERVAL_S)

    except KeyboardInterrupt:
        print()
        print("[Main] Ctrl+C received, stopping...")

    finally:
        csv_file.close()
        print("[Main] CSV closed.")


if __name__ == "__main__":
    main()
