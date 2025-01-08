"""Follow a path using APRIL tags as the waypoints."""
from apriltag_msgs.msg import AprilTagDetectionArray

import rclpy
import rclpy.logging
from rclpy.node import Node

from tf2_msgs.msg import TFMessage


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
        self._log.info(
            'Navigator started'
        )

    def _detections_cb(self, msg):
        """Handle a received Detections message."""
        if msg.detections:
            for detection in msg.detections:
                self._log.debug(
                    f'{detection.family}:{detection.id} ->'
                    f' ({detection.centre.x:.0f},{detection.centre.y:.0f})'
                )

    def _tf_cb(self, msg):
        """Handle a received TF message."""
        if msg.transforms:
            self._log.debug(
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
