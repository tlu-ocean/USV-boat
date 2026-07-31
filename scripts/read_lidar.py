#!/usr/bin/env python3

import serial


PORT = "/dev/ttyAMA3"
BAUD = 115200


def main() -> None:
    print(f"Reading TFmini Plus from {PORT} at {BAUD} baud")

    with serial.Serial(PORT, BAUD, timeout=1) as lidar:
        while True:
            # TFmini Plus standard frame:
            # 0x59 0x59 Dist_L Dist_H Strength_L Strength_H Temp_L Temp_H Checksum
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

            if sum(frame[:8]) & 0xFF != frame[8]:
                continue

            distance_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            temperature_c = (
                (frame[6] | (frame[7] << 8)) / 8.0 - 256.0
            )

            print(
                f"distance={distance_cm / 100:.2f} m, "
                f"strength={strength}, "
                f"temperature={temperature_c:.1f} °C"
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")
