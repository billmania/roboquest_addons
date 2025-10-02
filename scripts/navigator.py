#!/usr/bin/env python3

"""Follow a path using APRIL tags as the waypoints."""

from typing import List

from geometry_msgs.msg import TwistStamped

import rclpy
import rclpy.logging
from rclpy.node import Node

from rq_msgs.srv import Control

from tf2_msgs.msg import TFMessage

CLOSE_ENOUGH = 10

TURN_SPEED = 0.5  # radians per second
MOVE_SPEED = 0.1  # meters per second
MOVE_PERIOD = 0.1  # move update period in seconds
MIN_X = 0.05  # threshold for turning in meters
MIN_Z = 0.5   # no closer than these meters
WAYPOINT_FILE = '/opt/persist/tags/waypoints.txt'


class Navigation(Node):
    """Guide the robot toward a goal.

    Consume APRIL tag detections and TF to move toward
    an APRIL tag.
    """

    def __init__(self):
        """Create the subscriptions and publishers.

        Subscribe to TF transform messages. Setup a move timer to periodically
        Publish TwistStamped to the cmd_vel topic. Load the ordered list of
        APRIL tags to pursue.

        Loop through the tags in order.

        If the current tag is detected, by appearing in a TF message, set the
        TwistStamped attributes to move toward it. Upon moving to within
        CLOSE_ENOUGH of the current tag, proceed to the next tag.

        If the current tag isn't detected, execute the find_tag() method. If
        it's not found, proceed to the next tag.
        """
        super().__init__('navigation')

        self.shutdown = False
        self._log = self.get_logger()
        self._twist = TwistStamped()
        self._control_request = Control.Request()
        self._control_finished = False

        self._tags = self._get_tags()
        self._setup_topics()
        self.context.on_shutdown(self._cleanup_cb)
        self._setup_timers()
        self._setup_motor_control()
        self.set_motors('ON')

        self._log.info(
            'Navigator started'
        )

    def _get_tags(self) -> List[str]:
        """Return an ordered list of tags.

        Read the tags from a file. Each tag is identified by the
        string representation of its frame name. The order of the
        tags in the file is the order in which they should be
        treated as waypoints along a path.
        """
        tag_frames = []
        try:
            with open(WAYPOINT_FILE, 'r') as tags:
                while True:
                    tag_frame = tags.readline()
                    if not tag_frame:
                        break

                    tag_frames.append(tag_frame.strip())

        except Exception as e:
            self._log.warn(
                f'Exception reading tags file {WAYPOINT_FILE}: {e}'
            )

        self._log.debug(
            f'Waypoint tags: {tag_frames}'
        )

        return tag_frames

    def _setup_timers(self):
        """Create the timers for the methods doing the work.

        Timer callbacks are executed immediately upon the creation
        of the timer (in ROS Humble).
        """
        self._timers = []
        move_timer = self.create_timer(
            timer_period_sec=MOVE_PERIOD,
            callback=self._move
        )
        self._timers.append(move_timer)
        self._log.debug(
            '_move_timer setup'
        )

    def _trigger_shutdown(self) -> None:
        """Cause a shutdown to be triggered."""
        self._log.warn(
            'Triggering a shutdown'
        )
        rclpy.shutdown()

    def _setup_topics(self):
        """Create the subscribers and publishers."""
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

    def _setup_motor_control(self) -> None:
        """Create a client for the motor control service."""
        self._control_client = self.create_client(
            Control,
            'control_hat'
        )
        self._log.debug(
            'Connecting to control_hat service'
        )
        while not self._control_client.wait_for_service(timeout_sec=5.0):
            self._log.warn(
                'control_hat service not available'
            )
        self._log.debug(
            'Connected to control_hat service'
        )

    def set_motors(self, motor_state: str = 'OFF') -> None:
        """Set the state of the motors."""
        self._control_request.set_motors = motor_state
        self._log.debug(
                f'Setting motors {motor_state}'
        )
        self._control_response = None
        self._control_future = self._control_client.call_async(
            self._control_request
        )
        if self._control_future.done():
            try:
                self._control_finished = self._control_future.result()

            except Exception as e:
                self._log.warn(
                    f'control_hat excepted: {e}'
                )
        else:
            self._log.debug(
                'set_motors() still waiting for result'
            )
            self._control_future.add_done_callback(self._control_done)

    def _control_done(self, future):
        """Do the needful with the control future."""
        self._log.debug(f'control_done {future}')
        self._control_finished = future.result().success

    def _move(self):
        """Move the robot."""
        if not self._control_finished:
            self._log.warn(
                'motors not yet ON'
            )
            return

        self._cmd_vel_pub.publish(self._twist)

    def _tf_cb(self, msg):
        """Handle a received TF message."""
        if msg.transforms:
            transform = msg.transforms[0]
            self._log.debug(
                f'Received tf at {transform.header.stamp.sec}'
                f' {transform.child_frame_id}'
                f' -> {transform.header.frame_id}'
                f' x:{transform.transform.translation.x:0.3f}'
                f', y:{transform.transform.translation.y:0.3f}'
                f', z:{transform.transform.translation.z:0.3f}',
                throttle_duration_sec=0.1
            )

            if abs(transform.transform.translation.x) > MIN_X:
                if transform.transform.translation.x < 0.0:
                    # TURN_LEFT
                    self._twist.twist.angular.z = TURN_SPEED
                else:
                    # TURN_RIGHT
                    self._twist.twist.angular.z = -TURN_SPEED
            else:
                self._twist.twist.angular.z = 0.0

            if transform.transform.translation.z > MIN_Z:
                self._twist.twist.linear.x = MOVE_SPEED
            else:
                self._twist.twist.linear.x = 0.0

    def _cleanup_cb(self) -> None:
        """Perform a clean shutdown of the node's resources."""
        self._log.info(
            'Shutdown being executed'
        )
        for timer in self._timers:
            timer.cancel()

        self.set_motors('OFF')
        self.shutdown = True


def main(args=None):
    """Run the main loop."""
    rclpy.init(args=args)
    rclpy.logging.set_logger_level(
        'navigator',
        rclpy.logging.LoggingSeverity.DEBUG
    )

    navigation = Navigation()
    while not navigation.shutdown:
        try:
            rclpy.spin_once(timeout_sec=0.02)

        except Exception as e:
            print(
                f'spin_once() excepted: {e}'
            )

    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
