#!/usr/bin/env python3
"""Example temperature publisher node.

A ROS node to publish temperature readings from an MLX90614 I2C
temperature sensor.
"""

from sys import exit as OSexit

import RPi.GPIO as GPIO

from mlx90614 import MLX90614

import rclpy
import rclpy.logging
from rclpy.node import Node

from smbus2 import SMBus

from std_msgs.msg import Float32


I2C_ENABLE_PIN = 17
TIMER_PERIOD_S = 5
MILLIS_PER_S = 1000


class MLX90614Publisher(Node):
    """Publisher.

    The class handling the setup and operation of the MLX90614.
    """

    def __init__(self):
        """Create the publisher and the timer."""
        super().__init__('mlx90614_publisher')
        rclpy.logging.set_logger_level(
            'mlx90614_publisher',
            rclpy.logging.LoggingSeverity.DEBUG
        )
        self.get_logger().info('mlx90614_publisher starting')

        self._publisher_ = self.create_publisher(Float32, 'temperature', 1)
        self.i = 0
        self._bus = SMBus(6)
        self._mlx90614 = MLX90614(self._bus, address=0x5a)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(I2C_ENABLE_PIN, GPIO.OUT)
        GPIO.output(I2C_ENABLE_PIN, GPIO.HIGH)
        self.get_logger().debug('GPIO setup complete')

        self.timer = self.create_timer(TIMER_PERIOD_S, self._timer_callback)

    def _timer_callback(self):
        msg = Float32()
        try:
            self.get_logger().debug('calling get_amb_temp()')
            msg.data = self._mlx90614.get_amb_temp()
        except OSError as e:
            self.get_logger().warn(f'OSError in _timer_callback(): {e}')
            OSexit(1)
        else:
            self._publisher_.publish(msg)

    def shutdown(self):
        """Shutdown the connections."""
        GPIO.output(I2C_ENABLE_PIN, GPIO.LOW)
        GPIO.close()
        GPIO.cleanup()
        self._bus.close()


def main(args=None):
    """Initialize the ROS node.

    Initialize the node and instantiate the ExamplePublisher class.
    """
    rclpy.init(args=args)

    mlx90614_publisher = MLX90614Publisher()

    rclpy.spin(mlx90614_publisher)
    mlx90614_publisher.get_logger().debug('shutdown')

    mlx90614_publisher.shutdown()
    mlx90614_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
