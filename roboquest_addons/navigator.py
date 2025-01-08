#!/usr/bin/env python3

"""Follow a path using APRIL tags as the waypoints."""
from apriltag_msgs.msg import AprilTagDetectionArray

from geometry_msgs.msg import TwistStamped

import rclpy
import rclpy.logging
from rclpy.node import Node

from rq_msgs.srv import Control

from tf2_msgs.msg import TFMessage

CLOSE_ENOUGH = 10
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class Navigation(Node):
    """Guide the robot toward a goal.

    Consume APRIL tag detections and TF to move toward
    an APRIL tag.
    """

    def __init__(self):
        """Create the two subscriptions and publishers.

        Subscribe to APRIL tag detection messages and TF
        transform messages. Publish TwistStamped to the cmd_vel
        topic.
        """
        super().__init__('navigation')
        self._log = self.get_logger()

        self._twist = TwistStamped()
        self._control_request = Control.Request()

        self._camera = {
            'x_center': int(CAMERA_WIDTH / 2),
            'y_center': int(CAMERA_HEIGHT / 2)
        }

        self._detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            'detections',
            self._detections_cb,
            1
        )
        self._tf_sub = self.create_subscription(
            TFMessage,
            'tf',
            self._tf_cb,
            1
        )
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            1
        )
        self._control_client = self.create_client(
            Control,
            'control_hat'
        )
        self._log.info(
            'Connecting to control_hat service'
        )
        while not self._control_client.wait_for_service(timeout_sec=5.0):
            self._log.warn(
                'control_hat service not available'
            )
        self._control_request.set_motors = 'ON'
        self._log.info(
                'Requesting motors ON async'
        )
        self._control_response = None
        self._control_future = self._control_client.call_async(
            self._control_request
        )
        self._log.info(
            'Requested motors ON'
        )

        self._cmd_vel_timer = self.create_timer(
            0.5,
            self._move
        )
        self._log.info(
            'cmd_vel timer started'
        )
        self._log.info(
            'Navigator started'
        )

    def _move(self):
        """Move the robot."""
        if not self._control_future.done():
            self._log.info(
                'motors not yet ON'
            )
            return

        self._cmd_vel_pub.publish(self._twist)
        self._log.info(
            '_move():'
            f' {self._twist.twist.angular.z}'
        )

    def _detections_cb(self, msg):
        """Handle a received Detections message."""
        if msg.detections:
            x_diff = msg.detections[0].centre.x - self._camera['x_center']
            if abs(x_diff) <= CLOSE_ENOUGH:
                turn_toward = 'AHEAD'
                self._twist.twist.angular.z = 0.0
            elif x_diff < 0:
                turn_toward = 'TURN_LEFT'
                self._twist.twist.angular.z = 0.2
            else:
                turn_toward = 'TURN_RIGHT'
                self._twist.twist.angular.z = -0.2

            self._log.info(
                f'{turn_toward}'
            )
        else:
            turn_toward = 'LOST'
            self._twist.twist.angular.z = 0.0

    def _tf_cb(self, msg):
        """Handle a received TF message."""
        if msg.transforms:
            self._log.info(
                'Received tf'
            )


def main(args=None):
    """Run the main loop."""
    rclpy.init(args=args)
    rclpy.logging.set_logger_level(
        'navigator',
        rclpy.logging.LoggingSeverity.DEBUG
    )

    navigation = Navigation()
    rclpy.spin(navigation)
    navigation._control_request.set_motors = 'ON'
    navigation._control_client.call_async(navigation._control_request)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
