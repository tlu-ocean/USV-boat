#!/usr/bin/env python3

import time

import bme280
import smbus2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    Temperature,
    RelativeHumidity,
    FluidPressure,
)


class Bme280Node(Node):

    def __init__(self):
        super().__init__("bme280_node")

        # Parameters
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x76)
        self.declare_parameter("frame_id", "bme280_link")
        self.declare_parameter("publish_rate_hz", 1.0)

        self.i2c_bus_number = (
            self.get_parameter("i2c_bus")
            .get_parameter_value()
            .integer_value
        )

        self.address = (
            self.get_parameter("address")
            .get_parameter_value()
            .integer_value
        )

        self.frame_id = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )

        publish_rate_hz = (
            self.get_parameter("publish_rate_hz")
            .get_parameter_value()
            .double_value
        )

        # Publishers
        self.temperature_pub = self.create_publisher(
            Temperature,
            "/environment/air_temperature",
            10,
        )

        self.humidity_pub = self.create_publisher(
            RelativeHumidity,
            "/environment/relative_humidity",
            10,
        )

        self.pressure_pub = self.create_publisher(
            FluidPressure,
            "/environment/air_pressure",
            10,
        )

        # Sensor
        self.bus = None
        self.calibration_params = None

        self.last_connect_attempt = 0.0
        self.reconnect_interval_s = 2.0

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.read_and_publish,
        )

        self.get_logger().info(
            "BME280 node started"
        )

    def connect_sensor(self):

        now = time.monotonic()

        if (
            now - self.last_connect_attempt
            < self.reconnect_interval_s
        ):
            return

        self.last_connect_attempt = now

        try:
            self.get_logger().info(
                f"Connecting to BME280 on I2C "
                f"bus {self.i2c_bus_number}, "
                f"address 0x{self.address:02X}"
            )

            self.bus = smbus2.SMBus(
                self.i2c_bus_number
            )

            self.calibration_params = (
                bme280.load_calibration_params(
                    self.bus,
                    self.address,
                )
            )

            self.get_logger().info(
                "BME280 connected"
            )

        except Exception as exc:

            self.get_logger().warning(
                f"BME280 connection failed: {exc}"
            )

            if self.bus is not None:
                try:
                    self.bus.close()
                except Exception:
                    pass

            self.bus = None
            self.calibration_params = None

    def read_and_publish(self):

        if (
            self.bus is None
            or self.calibration_params is None
        ):
            self.connect_sensor()
            return

        try:

            sample = bme280.sample(
                self.bus,
                self.address,
                self.calibration_params,
            )

            temperature_c = float(
                sample.temperature
            )

            humidity_pct = float(
                sample.humidity
            )

            pressure_hpa = float(
                sample.pressure
            )

        except Exception as exc:

            self.get_logger().error(
                f"BME280 read error: {exc}"
            )

            try:
                self.bus.close()
            except Exception:
                pass

            self.bus = None
            self.calibration_params = None
            return

        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # Temperature: °C
        temperature_msg = Temperature()
        temperature_msg.header.stamp = stamp
        temperature_msg.header.frame_id = self.frame_id
        temperature_msg.temperature = temperature_c
        temperature_msg.variance = 0.0

        self.temperature_pub.publish(
            temperature_msg
        )

        # RelativeHumidity expects 0.0 - 1.0
        humidity_msg = RelativeHumidity()
        humidity_msg.header.stamp = stamp
        humidity_msg.header.frame_id = self.frame_id
        humidity_msg.relative_humidity = (
            humidity_pct / 100.0
        )
        humidity_msg.variance = 0.0

        self.humidity_pub.publish(
            humidity_msg
        )

        # FluidPressure expects Pa
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = stamp
        pressure_msg.header.frame_id = self.frame_id
        pressure_msg.fluid_pressure = (
            pressure_hpa * 100.0
        )
        pressure_msg.variance = 0.0

        self.pressure_pub.publish(
            pressure_msg
        )

    def destroy_node(self):

        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = Bme280Node()

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
