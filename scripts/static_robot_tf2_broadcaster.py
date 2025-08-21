#!/usr/bin/env python3
"""Publish a static transform for the robot."""
import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    """
    Convert Euler angles.

    A utility function to conver from Euler angles to a quaternion.
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Publish transforms that never change.

    Publish transforms from `world` to a static robot frame.
    The transforms are only published once at startup and are constant for all
    time.
    """

    def __init__(self, transformation):
        """Initialize the class."""
        super().__init__(
            'static_robot_tf2_broadcaster',
            enable_logger_service=True
        )

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        """Make the transforms."""
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = transformation[1]

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]),
            float(transformation[6]),
            float(transformation[7])
        )
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)


def main():
    """Run the main logic."""
    logger = rclpy.logging.get_logger('logger')

    if len(sys.argv) != 8:
        logger.error(
            'Usage: \n'
            ' ros2 run roboquest_addons static_robot_tf2_broadcaster'
            ' child_frame_name x y z roll pitch yaw'
        )
        sys.exit(1)

    if sys.argv[1] == 'world':
        logger.error('Static robot frame cannot be "world"')
        sys.exit(2)

    rclpy.init()
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
