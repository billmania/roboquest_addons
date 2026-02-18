#!/usr/bin/env python3

"""Follow a path.

Use the camera and APRIL tags as the waypoints of the
path. Use the LiDAR to detect _obstacles.
"""

from enum import Enum
from math import atan2, degrees, isinf, isnan, pi
from time import time
from traceback import print_exc
from typing import List, Tuple, Union

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped

import rclpy
import rclpy.logging
from rclpy.node import Node

from rq_msgs.srv import Control

from sensor_msgs.msg import LaserScan

from tf2_msgs.msg import TFMessage

TURN_SPEED = 0.5  # radians per second
MOVE_SPEED = 0.1  # meters per second
MOVE_PERIOD = 0.1  # move update period in seconds
MAX_SEARCH_TIME = round((2 * pi) / TURN_SPEED)
MAX_OBSTACLE_RANGE = 15.0
MAX_TAG_DISTANCE_M = 5.0  # max detect distance for tags
MAX_TAG_BEARING_RAD = (pi / 4)  # max detect bearing for tags
MAX_TAG_LOST_S = 0.5
OBSTACLE_CLOSE_ENOUGH = 0.3  # minimum proximity to obstacles
IGNORE_RANGES = 1  # after sorting, ignore these lowest ranges
MEAN_RANGES = 3  # after ignoring, average these lowest ranges
TAG_TRANS_TRIM = 0.0  # added to translation.z
TAG_TRANS_FACTOR = 1.0  # multiplied by translation.z
TAG_CLOSE_ENOUGH = OBSTACLE_CLOSE_ENOUGH * 2.0
AVOIDANCE_DIST_M = (2 * OBSTACLE_CLOSE_ENOUGH)
AVOIDANCE_CYCLES = 10
AHEAD_ANGLE_DEG = 10  # looking ahead
SIDE_ANGLE_DEG = 20  # looking to the side
PERSIST_DIR = (
    '/usr/src/ros2ws/install'
    '/roboquest_addons/share/roboquest_addons'
    '/persist'
)
WAYPOINT_FILE = PERSIST_DIR + '/tags/waypoints.txt'


class State(Enum):
    """Specify the robot state."""

    START = 0
    ESTOP = 1  # stop moving, never move again
    AVOIDING = 2  # adjusting motion to avoid obstacle(s)
    PURSUING = 3  # moving toward the detected tag
    SEARCHING = 4  # searching for the tag
    DONE = 6


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
        TwistStamped attributes to move toward it.

        If the current tag isn't detected, execute the find_tag() method. If
        it's not found, proceed to the next tag.
        """
        super().__init__('navigation')

        self._log = self.get_logger()
        self._state = State.START
        self._twist = TwistStamped()
        self._control_request = Control.Request()
        self._control_future = None
        self._control_finished = False
        self._searching_start = None
        self._obstacle_range_max = MAX_OBSTACLE_RANGE
        self._obstacles = {
            'ahead': None,
            'left': None,
            'right': None
        }
        self._avoidance_cycles = AVOIDANCE_CYCLES
        self._fov_half_ahead = None

        self._tags = self._get_tags()
        self._get_next_tag()
        self._setup_topics()
        self.context.on_shutdown(self.cleanup_cb)
        self._setup_motor_control()
        self.set_motors('ON')

        self._log.info(
            f'\nMAX_SEARCH_TIME = {MAX_SEARCH_TIME}'
            f'\nTAG_TRANS_TRIM = {TAG_TRANS_TRIM}'
            f'\nTAG_TRANS_FACTOR = {TAG_TRANS_FACTOR}'
        )
        if self._camera_frames():
            self._set_state(State.SEARCHING)
        else:
            self._set_state(State.DONE)
        self._setup_timers()
        self._log.info(
            'Navigator started'
        )

    def _camera_frames(self) -> bool:
        """Check if camera frames are published."""
        return True

    def _get_next_tag(self) -> None:
        """Get the next tag in the list.

        If there aren't any more tags self._tag['name']
        is set to None. Otherwise the timestamp and
        poofs_left are reset. range and bearing won't be
        set until the tag is detected.
        """
        self._tag = {
            'name': None,
            'range': None,
            'bearing': None,
            'timestamp': None
        }

        try:
            tag_name = self._tags.pop(0)

        except IndexError:
            self._log.info(
                'No more tags'
            )
            return

        else:
            if tag_name != '':
                self._log.info(
                    f'Looking for {tag_name}'
                )
                self._tag['name'] = tag_name

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
        self._scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self._scan_cb,
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

    def _estop(self):
        """Stop everything."""
        pass

    def _searching(self):
        """Search for the current tag."""
        if self._searching_start:
            if (time() - self._searching_start) >= MAX_SEARCH_TIME:
                self._stop_rotating()
                self._log.warning(
                    'Tag not found'
                )
                self._get_next_tag()
                if not self._tag['name']:
                    self._set_state(State.DONE)

                return

        if self._tag['timestamp']:
            self._log.debug(
                f"Found tag {self._tag['name']}"
            )
            self._stop_rotating()
            self._set_state(State.PURSUING)
        else:
            self._rotate_to_search()

    def _pursuing(self):
        """Move toward the tag.

        Move to within TAG_CLOSE_ENOUGH of the tag. The tag
        will also be identified as an obstacle, so check the distance
        to the tag before checking the distance to _obstacles.
        """
        tag_range = self._tag['range']
        obstacle_range = min([
            v for v in self._obstacles.values() if type(v) is float
            ] + [self._obstacle_range_max]
        )
        self._log.debug(
            '_pursuing:'
            f' tag: {tag_range:0.3f}'
            f' obstacle: {obstacle_range:0.3f}',
            throttle_duration_sec=0.5
        )
        if type(tag_range) is not float:
            self._log.warning(
                '_pursuing: Missing tag range'
            )
            return

        if tag_range <= TAG_CLOSE_ENOUGH:
            self._log.debug(
                'Close enough to tag'
            )
            # TODO: Add a state for "found the tag"
            self._get_next_tag()
            if self._tag['name']:
                self._set_state(State.SEARCHING)
            else:
                self._set_state(State.DONE)
            return

        try:
            if obstacle_range <= OBSTACLE_CLOSE_ENOUGH:
                self._log.debug(
                    'Closed enough to obstacle'
                )
                self._set_state(State.AVOIDING)
                return

        except Exception as e:
            self._log.debug(
                f'_pursuing: exception {e}'
            )

        if (time() - self._tag['timestamp']) >= MAX_TAG_LOST_S:
            self._log.warning(
                f"Tag {self._tag['name']} lost"
            )
            self._tag['timestamp'] = None
            self._tag['range'] = None
            self._tag['bearing'] = None
            self._set_state(State.SEARCHING)
            return

        # TODO: Handle un-updated range and bearing better
        self._twist.twist.linear.x = (
            min(1.0, tag_range / MAX_TAG_DISTANCE_M) * MOVE_SPEED
        )
        self._twist.twist.angular.z = (
            min(1.0, self._tag['bearing'] / MAX_TAG_BEARING_RAD) * TURN_SPEED
        )

    def _avoiding(self):
        """Move around obstacles.

        Rotate away from the closest obstacle until all
        three obstacle ranges are greater than
        OBSTACLE_CLOSE_ENOUGH, then move forward for
        AVOID_DISTANCE_M.
        """
        if not self._avoidance_cycles:
            #
            # Traveled adequate linear distance.
            #
            self._avoidance_cycles = AVOIDANCE_CYCLES
            self._set_state(State.SEARCHING)
            return

        if (
            (not self._obstacles['left']
             or self._obstacles['left'] > OBSTACLE_CLOSE_ENOUGH)
            and (not self._obstacles['ahead']
                 or self._obstacles['ahead'] > OBSTACLE_CLOSE_ENOUGH)
            and (not self._obstacles['right']
                 or self._obstacles['right'] > OBSTACLE_CLOSE_ENOUGH)
        ):
            #
            # Rotated enough, now move forward.
            #
            self._twist.twist.linear.x = MOVE_SPEED
            self._twist.twist.angular.z = 0.0
            self._avoidance_cycles -= 1
            return

        self._twist.twist.linear.x = 0.0
        if (
            not self._obstacles['left']
            or (self._obstacles['right']
                and self._obstacles['left'] > self._obstacles['right'])
        ):
            #
            # The right obstacle is closer than the left, so
            # rotate to the left.
            #
            self._twist.twist.angular.z = TURN_SPEED
        else:
            self._twist.twist.angular.z = -TURN_SPEED

    def _done(self):
        """Be done."""
        pass

    def _set_state(self, new_state: State = None) -> None:
        """Set a new state."""
        if new_state and self._state != new_state:
            self._log.debug(
                'Changing state'
                f' from {self._state.name}'
                f' to {new_state.name}'
            )
            self._state = new_state

    def _move(self):
        """Move the robot.

        This method is expected to be called periodically.
        It uses self._state, the distance to any detected
        tag, and the distance to any obstacles, to decide
        in which direction to move and then move.
        """
        if not self._control_finished:
            #
            # The drive motors have not yet been enabled.
            #
            self._log.warning(
                'Still waiting for motors to be enabled',
                throttle_duration_sec=1.0
            )
            return

        if self._state == State.ESTOP:
            pass

        elif self._state == State.SEARCHING:
            self._searching()

        elif self._state == State.PURSUING:
            self._pursuing()

        elif self._state == State.AVOIDING:
            self._avoiding()

        elif self._state == State.DONE:
            self._done()

        else:
            self._log.warning(
                'Unrecognized state',
                throttle_duration_sec=15.0
            )

        try:
            self._cmd_vel_pub.publish(self._twist)

        except Exception as e:
            self._log.warning(
                f'cmd_vel publish excepted {e}'
            )

    def _rotate_to_search(self):
        """Rotate the robot.

        Rotate the robot, with no linear motion, to look for a
        tag.
        """
        self._twist.twist.angular.z = TURN_SPEED
        self._twist.twist.linear.x = 0.0
        if not self._searching_start:
            self._searching_start = time()

    def _stop_rotating(self):
        """Stop rotating the robot."""
        self._twist.twist.angular.z = 0.0
        self._twist.twist.linear.x = 0.0
        self._searching_start = None

    def _get_detected_tag(
       self,
       transforms: TFMessage) -> Union[TransformStamped, None]:
        """Return the detected tag.

        At any time, the robot is searching for a specific tag. This
        method returns a "detected" tag if it matches the tag wanted
        by the robot.
        """
        if not transforms:
            return None

        for transform in transforms:
            if transform.child_frame_id == self._tag['name']:
                return transform

        return None

    def _obstacle_distance(
         self,
         range_series: List[float]) -> float:
        """Determine the distance to the nearest obstacle."""
        min_ranges = [
            point for point in range_series
            if not isinf(point) and not isnan(point)
        ]
        ranges = sorted(min_ranges)[IGNORE_RANGES:][:MEAN_RANGES]
        if len(ranges) > 0:
            return sum(ranges) / len(ranges)
        else:
            return None

    def _get_fov_points(
        self,
        degrees_per_point: float,
        ahead_angle_deg: int,
        side_angle_deg: int
       ) -> Tuple[int, int]:
        """Calculate the four FOV edges.

        Using ahead_angle_deg and side_angle_deg calculate
        the length of half of the ahead FOV and the length
        of a full side FOV.
        """
        half_ahead_points = round((ahead_angle_deg // 2) / degrees_per_point)
        side_points = round(side_angle_deg / degrees_per_point)
        return (
            half_ahead_points,
            side_points
        )

    def _scan_cb(self, msg):
        """Handle a received LiDAR message.

        Determine the distance to any obstacles ahead or to the
        sides of the robot.

        The RPLiDAR has a triangle on the top, which points forward
        (aligned with the robot's positive X axis. The LiDAR range
        point aligned with that triangle is "0". The points then
        progress as a positive rotation around the robot's Z axis.

        As a result, the LiDAR's four most forward-pointing range
        points are 1, 0, 719, and 718.
        """
        if not self._fov_half_ahead:
            try:
                self._obstacle_range_max = msg.range_max
                self._log.debug(
                    f'Range min: {msg.range_min:0.3f}'
                    f', Range max: {self._obstacle_range_max:0.3f}'
                )
                self._fov_half_ahead, self._fov_side = self._get_fov_points(
                    degrees(msg.angle_increment),
                    AHEAD_ANGLE_DEG,
                    SIDE_ANGLE_DEG
                )

            except Exception as e:
                self._log.warning(
                    f'_scan_cb 1 excepted {e}'
                )
                return

            self._log.debug(
                'FOV slices'
                f' Ahead :{self._fov_half_ahead} + -{self._fov_half_ahead}:'
                f', Left {self._fov_half_ahead}'
                f':{self._fov_half_ahead+self._fov_side}'
                f', Right {-(self._fov_half_ahead+self._fov_side)}:'
                f'{-self._fov_half_ahead}'
            )

        try:
            self._obstacles['ahead'] = self._obstacle_distance(
                msg.ranges[
                    :self._fov_half_ahead
                ] +
                msg.ranges[
                    -self._fov_half_ahead:
                ]
            )

            self._obstacles['left'] = self._obstacle_distance(
                msg.ranges[
                    self._fov_half_ahead:self._fov_half_ahead+self._fov_side
                ]
            )

            self._obstacles['right'] = self._obstacle_distance(
                msg.ranges[
                    -(self._fov_half_ahead+self._fov_side):
                    -self._fov_half_ahead
                ]
            )

        except Exception as e:
            self._log.warning(
                f'_scan_cb 2 excepted {e}'
                f' {print_exc()}'
            )
            return

#         self._log.debug(
#             'Obstacles:'
#             f" {self._obstacles['left']:0.3f}"
#             f" {self._obstacles['ahead']:0.3f}"
#             f" {self._obstacles['right']:0.3f}",
#             throttle_duration_sec=0.5
#         )

    def _tf_cb(self, msg):
        """Handle a received TF message.

        Update the range and bearing to the tag, when it's detected.
        """
        #
        # Try to detect the current APRIL tag.
        #
        detected_tag = self._get_detected_tag(msg.transforms)

        if detected_tag:
            try:
                if not (
                    OBSTACLE_CLOSE_ENOUGH
                    < detected_tag.transform.translation.z <
                    MAX_TAG_DISTANCE_M
                ):
                    self._log.warning(
                        '_tf_cb: translation.z out of range at'
                        f' {detected_tag.transform.translation.z:0.3f}'
                    )
                    return

            except Exception as e:
                self._log.warn(
                    f'_tf_cb1 exception: {e}'
                )
                return

            try:
                self._tag['range'] = (
                    (detected_tag.transform.translation.z + TAG_TRANS_TRIM)
                    * TAG_TRANS_FACTOR
                )
                if self._tag['range'] <= OBSTACLE_CLOSE_ENOUGH:
                    self._log.warning(
                        '_tf_cb: translation.z = '
                        f'{detected_tag.transform.translation.z:0.3f}'
                    )
                bearing_offset = detected_tag.transform.translation.x
                self._tag['bearing'] = (
                    -(atan2(bearing_offset, self._tag['range']))
                )
                self._tag['timestamp'] = time()

            except Exception as e:
                self._log.warn(
                    f'_tf_cb2 exception: {e}'
                )
                return

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
            navigation._log.warn(
                f'spin() Exception: {e}'
                f'\n{print_exc()}'
            )


if __name__ == '__main__':
    main()
