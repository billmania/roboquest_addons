#!/usr/bin/env python3

"""Follow a path using APRIL tags as the waypoints."""

from math import pi
from time import time
from typing import List, Union

from geometry_msgs.msg import TransformStamped
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
MAX_RUNTIME = 300.0  # in seconds
MAX_ROTATE_TIME = int((2 * pi) / TURN_SPEED) + 1
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

        self._log = self.get_logger()
        self._twist = TwistStamped()
        self._control_request = Control.Request()
        self._control_future = None
        self._control_finished = False
        self._rotation_start = None

        self._setup_timers()
        self._tags = self._get_tags()
        self._current_tag = self._get_next_tag()
        self._setup_topics()
        self.context.on_shutdown(self.cleanup_cb)
        self._setup_motor_control()
        self.set_motors('ON')

        self._log.info(
            'Navigator started'
        )

    def _get_next_tag(self):
        """Get the next tag in the list."""
        try:
            next_tag = self._tags.pop(0)
            self._log.debug(
                f'Looking for {next_tag}'
            )
            return next_tag

        except IndexError:
            return None

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
            rclpy.spin_once(
                node=self,
                executor=None,
                timeout_sec=0.0
            )
        self._log.debug(
            'Connected to control_hat service'
        )

    def set_motors(self, motor_state: str = 'OFF') -> None:
        """Set the state of the motors."""
        if self._control_future:
            #
            # Still waiting for the previous service call to complete.
            #
            return

        self._control_request.set_motors = motor_state
        self._log.info(
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
            self._control_future.add_done_callback(self._control_done)

    def _control_done(self, future):
        """Do the needful with the control future."""
        if future.result().success:
            self._log.debug('set_motors service call succeeded')
            self._control_finished = True
        else:
            self._log.warn('set_motors service call failed')
            self._control_finished = False

        self._control_future = None

    def _move(self):
        """Move the robot.

        This method is expected to be called continuously,
        regardless of any tag being detected.
        """
        if not self._control_finished:
            return

        self._cmd_vel_pub.publish(self._twist)

    def _rotate_robot(self):
        """Rotate the robot.

        Rotate the robot, with no linear motion, to look for a
        tag.
        """
        self._twist.twist.angular.z = TURN_SPEED
        self._twist.twist.linear.x = 0.0
        if not self._rotation_start:
            self._rotation_start = time()

    def _stop_rotating(self):
        """Stop rotating the robot."""
        self._twist.twist.angular.z = 0.0
        self._twist.twist.linear.x = 0.0
        self._rotation_start = None

    def _get_detected_tag(
       self,
       transforms: TFMessage) -> Union[TransformStamped, None]:
        """Return the detected tag."""
        if not transforms:
            return None

        for transform in transforms:
            if transform.child_frame_id == self._current_tag:
                return transform

        return None

    def _tf_cb(self, msg):
        """Handle a received TF message.

        It controls the duration of rotation to find a tag. When
        maximum rotation time is reached, rotation will be stopped.
        If no tag is detected, the method returns. If any tag is
        detected, then a check is made for the current tag.
        """
        detected_tag = self._get_detected_tag(msg.transforms)

        if not detected_tag and not self._rotation_start:
            self._log.debug('Starting rotation to find a tag')
            self._rotate_robot()
            return

        if not detected_tag and self._rotation_start:
            if (time() - self._rotation_start) >= MAX_ROTATE_TIME:
                self._stop_rotating()
                self._log.warn(
                    'Maximum rotation time reached.'
                    f' Failed to find tag {self._current_tag}'
                )

                self._current_tag = self._get_next_tag()
                if not self._current_tag:
                    self._log.warn(
                        'No more tags to find'
                    )
                    self._begin_shutdown()

            return

        if detected_tag and self._rotation_start:
            self._log.debug('Stopping rotation')
            self._stop_rotating()

        self._log.debug(
            f'Received tf at {detected_tag.header.stamp.sec}'
            f' {detected_tag.child_frame_id}'
            f' x:{detected_tag.transform.translation.x:0.3f}'
            f', z:{detected_tag.transform.translation.z:0.3f}',
            throttle_duration_sec=1.0
        )

        if abs(detected_tag.transform.translation.x) > MIN_X:
            if detected_tag.transform.translation.x < 0.0:
                # TURN_LEFT
                self._twist.twist.angular.z = TURN_SPEED
            else:
                # TURN_RIGHT
                self._twist.twist.angular.z = -TURN_SPEED
        else:
            self._twist.twist.angular.z = 0.0

        if detected_tag.transform.translation.z > MIN_Z:
            self._twist.twist.linear.x = MOVE_SPEED
        else:
            self._twist.twist.linear.x = 0.0
            self._current_tag = self._get_next_tag()
            if not self._current_tag:
                self._log.info(
                    'Found last tag'
                )
                self._begin_shutdown()

    def _begin_shutdown(self) -> None:
        """Prepare for the node to be shutdown."""
        self._log.warn(
            'Preparing to shutdown'
        )
        self.cleanup_cb()
        self.destroy_node()
        rclpy.shutdown()

    def cleanup_cb(self) -> None:
        """Cleanup quickly.

        Release/Destroy ROS resources quickly. Intended for use when
        the node has started the shutdown process.
        """
        self._log.info(
            'Executing cleanup_cb'
        )
        for timer in self._timers:
            timer.cancel()
        self.set_motors('OFF')
        self._control_client.destroy()

        #
        # In case the motor power isn't removed.
        #
        self._twist.twist.angular.z = 0.0
        self._twist.twist.linear.x = 0.0
        self._move()
        self._move()

        while self._control_future:
            rclpy.spin_once(
                node=self,
                executor=None,
                timeout_sec=0.1
            )

        self._log.info(
            'Cleanup completed'
        )


def main(args=None):
    """Run the main loop."""
    rclpy.init(args=args)
    rclpy.logging.set_logger_level(
        'navigator',
        rclpy.logging.LoggingSeverity.DEBUG
    )

    navigation = Navigation()
    while rclpy.ok():
        try:
            rclpy.spin(
                node=navigation
            )

        except Exception as e:
            navigation._log.warn(f'spin() Exception: {e}')


if __name__ == '__main__':
    main()
