#!/usr/bin/env python3

"""Follow a path using APRIL tags as the waypoints."""
from apriltag_msgs.msg import AprilTagDetectionArray

from geometry_msgs.msg import TwistStamped

import rclpy
import rclpy.logging
from rclpy.node import Node

from rq_msgs.srv import Control

from sensor_msgs.msg import CameraInfo

from tf2_msgs.msg import TFMessage

CLOSE_ENOUGH = 10

TURN_SPEED = 1.0  # radians per second
MOVE_SPEED = 0.1  # meters per second


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
            'x_center': None,
            'y_center': None
        }
        self._control_finished = False

        self._detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            'detections',
            self._detections_cb,
            1
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            'rq_camera_node0/camera_info',
            self._camera_info_cb,
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
        if self._control_future.done():
            try:
                self._log.info(
                    f'control_hat result: {self._control_future.result()}'
                )
                self._control_finished = self._control_future.result()

            except Exception as e:
                self._log.warn(
                    f'control_hat excepted: {e}'
                )
        else:
            self._log.warn(
                'control_hat still running'
            )
            self._control_future.add_done_callback(self._control_done)

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

    def _control_done(self, future):
        """Do the needful with the control future."""
        self._log.info(f'control_done {future}')
        self._control_finished = future.result().success

    def _move(self):
        """Move the robot."""
        if not self._control_finished:
            self._log.warn(
                'motors not yet ON'
            )
            return

        self._cmd_vel_pub.publish(self._twist)
        self._log.info(
            '_move():'
            f' {self._twist.twist.angular.z}'
        )

    def _camera_info_cb(self, msg):
        """Handle a single CameraInfo message."""
        self._log.info(
            '_camera_info_cb'
            f' width: {msg.width}'
            f', height: {msg.height}',
            throttle_duration_sec=60
        )
        self._camera = {
            'x_center': int(msg.width / 2),
            'y_center': int(msg.height / 2)
        }
        self.destroy_subscription(self._camera_info_sub)
        self._log.info(
            '_camera_info_sub destroyed'
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
                self._twist.twist.angular.z = TURN_SPEED
            else:
                turn_toward = 'TURN_RIGHT'
                self._twist.twist.angular.z = -TURN_SPEED

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
