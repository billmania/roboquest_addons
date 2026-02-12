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
MAX_RUNTIME = 300.0  # in seconds
MAX_ROTATE_TIME_S = round((2 * pi) / TURN_SPEED)
MAX_TAG_DISTANCE_M = 5.0  # max detect distance for tags
MAX_TAG_BEARING_RAD = (pi / 4)  # max detect bearing for tags
MAX_TAG_LOST_S = 0.5
MIN_OBSTACLE_DISTANCE_M = 0.3  # minimum proximity to obstacles
AVOIDANCE_DIST_M = (2 * MIN_OBSTACLE_DISTANCE_M)
AVOIDANCE_MAX_CYCLES = 10
MIN_Z = 0.4   # no closer than these meters
AHEAD_ANGLE_DEG = 10  # looking ahead
SIDE_ANGLE_DEG = 20  # looking to the side
MIN_POINTS = 10  # minimum required points to declare an obstacle
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

    def __init__(self, min_z: float):
        """Create the subscriptions and publishers.

        Subscribe to TF transform messages. Setup a move timer to periodically
        Publish TwistStamped to the cmd_vel topic. Load the ordered list of
        APRIL tags to pursue.

        Loop through the tags in order.

        If the current tag is detected, by appearing in a TF message, set the
        TwistStamped attributes to move toward it. Upon moving to within
        min_z of the current tag, proceed to the next tag.

        If the current tag isn't detected, execute the find_tag() method. If
        it's not found, proceed to the next tag.
        """
        super().__init__('navigation')

        self._log = self.get_logger()
        self._state = State.START
        self._twist = TwistStamped()
        self._control_request = Control.Request()
        self._min_z = min_z
        self._control_future = None
        self._control_finished = False
        self._searching_start = None
        self._obstacles = {
            'ahead': None,
            'left': None,
            'right': None
        }
        self._avoidance_max_cycles = AVOIDANCE_MAX_CYCLES
        self._fov_half_ahead = None

        self._set_state(State.SEARCHING)
        self._setup_timers()
        self._tags = self._get_tags()
        self._get_next_tag()
        self._setup_topics()
        self.context.on_shutdown(self.cleanup_cb)
        self._setup_motor_control()
        self.set_motors('ON')

        self._log.info(
            'Navigator started'
        )
        self._log.debug(
            f'MAX_ROTATE_TIME_S = {MAX_ROTATE_TIME_S}'
        )

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
            if (time() - self._searching_start) >= MAX_ROTATE_TIME_S:
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

        Calculate the error between the robot heading and the
        tag bearing. Slot the error into 'left', 'ahead', or 'right'.
        For a non-ahead error, calculate a proportional rotation rate.
        Check for an obstacle between the robot and the tag.

        Couple of things to consider:
        1. is the tag left, ahead, or right
        2. is the tag closer than any obstacles
        3. the range to the left, ahead, and right obstacles
        """
        try:
            if min(self._obstacles.values()) <= MIN_OBSTACLE_DISTANCE_M:
                self._set_state(State.AVOIDING)
                return
        except Exception:
            pass

        if (time() - self._tag['timestamp']) >= MAX_TAG_LOST_S:
            self._log.warning(
                f"Tag {self._tag['name']} lost"
            )
            self._tag['timestamp'] = None
            self._tag['range'] = None
            self._tag['bearing'] = None
            self._twist.twist.linear.x = 0.0
            self._twist.twist.angular.z = 0.0
            self._set_state(State.SEARCHING)
            return

        # TODO: Handle un-updated range and bearing better
        self._twist.twist.linear.x = (
            min(1.0, self._tag['range'] / MAX_TAG_DISTANCE_M) * MOVE_SPEED
        )
        self._twist.twist.angular.z = (
            min(1.0, self._tag['bearing'] / MAX_TAG_BEARING_RAD) * TURN_SPEED
        )

    def _avoiding(self):
        """Move around obstacles.

        Rotate away from the closest obstacle until all
        three obstacle ranges are greater than
        MIN_OBSTACLE_DISTANCE_M, then move forward for
        AVOID_DISTANCE_M.
        """
        if self._avoidance_max_cycles == 0:
            #
            # Traveled adequate linear distance.
            #
            self._avoidance_max_cycles = AVOIDANCE_MAX_CYCLES
            self._twist.twist.linear.x = 0.0
            self._twist.twist.angular.z = 0.0
            self._set_state(State.SEARCHING)
            return

        if (
            (not self._obstacles['left']
             or self._osbstacles['left'] > AVOIDANCE_DIST_M)
            and (not self._obstacles['ahead']
                 or self._obstacles['ahead'] > AVOIDANCE_DIST_M)
            and (not self._obstacles['right']
                 or self._obstacles['right'] > AVOIDANCE_DIST_M)
        ):
            #
            # Rotated enough, now move away.
            #
            self._twist.twist.linear.x = MOVE_SPEED
            self._twist.twist.angular.z = 0.0
            self._avoidance_max_cycles -= 1
            return

        if (
            not self._obstacles['left']
            or (self._obstacles['right']
                and self._obstacles['left'] > abs(self._obstacles['right']))
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
        ranges = sorted(min_ranges)[1:][:3]
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
            #
            # In case the minimum distance was set to less than the
            # capability of the LiDAR.
            #
            try:
                self._min_z = max(self._min_z, msg.range_min)
                self._log.debug(
                    f'Range min: {msg.range_min}'
                    f', Range max: {msg.range_max}'
                    f', MIN_Z: {self._min_z}'
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
#             throttle_duration_sec=1.0
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
                self._tag['range'] = detected_tag.transform.translation.z
                bearing_offset = detected_tag.transform.translation.x
                self._tag['bearing'] = (
                    -(atan2(bearing_offset, self._tag['range']))
                )
                self._tag['timestamp'] = time()

            except Exception as e:
                self._log.warn(
                    f'_tf_cb exception: {e}'
                )
                return

            self._log.debug(
                '_tf_cb:'
                f" bearing: {self._tag['bearing']:0.3f} rad"
                f" range: {self._tag['range']:0.3f} m"
            )

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

    navigation = Navigation(MIN_Z)
    while rclpy.ok():
        try:
            rclpy.spin(
                node=navigation
            )

        except Exception as e:
            navigation._log.warn(f'spin() Exception: {e}')


if __name__ == '__main__':
    main()
