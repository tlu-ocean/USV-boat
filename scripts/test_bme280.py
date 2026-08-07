import time

import board
import adafruit_bme280.basic as adafruit_bme280


BME280_ADDRESS = 0x76


def main():
    print("Opening I2C bus...")

    i2c = board.I2C()

    print(
        f"Connecting to BME280 at "
        f"0x{BME280_ADDRESS:02X}..."
    )

    sensor = adafruit_bme280.Adafruit_BME280_I2C(
        i2c,
        address=BME280_ADDRESS,
    )

    print("BME280 connected")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            temperature_c = sensor.temperature
            humidity_pct = sensor.relative_humidity
            pressure_hpa = sensor.pressure

            print(
                f"Temperature: {temperature_c:6.2f} C | "
                f"Humidity: {humidity_pct:6.2f} % | "
                f"Pressure: {pressure_hpa:7.2f} hPa"
            )

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
