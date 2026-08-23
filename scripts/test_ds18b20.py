import time
from pathlib import Path


DEVICE_ID = "28-000000230f20"

DEVICE_FILE = Path(
    f"/sys/bus/w1/devices/{DEVICE_ID}/w1_slave"
)


def read_temperature_c():
    text = DEVICE_FILE.read_text()

    lines = text.strip().splitlines()

    if len(lines) < 2:
        raise RuntimeError("Unexpected DS18B20 data format")

    if not lines[0].strip().endswith("YES"):
        raise RuntimeError("DS18B20 CRC check failed")

    marker = "t="

    if marker not in lines[1]:
        raise RuntimeError("Temperature value not found")

    temperature_milli_c = int(
        lines[1].split(marker)[1]
    )

    return temperature_milli_c / 1000.0


def main():
    print(f"DS18B20 device: {DEVICE_ID}")
    print(f"Reading: {DEVICE_FILE}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            temperature_c = read_temperature_c()

            print(
                f"Temperature: {temperature_c:.3f} C"
            )

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()

