#!/usr/bin/env python
"""Test implementation of ROS2 dynamic parameters."""
from rcl_interfaces.msg import SetParametersResult

import rclpy
import rclpy.node
from rclpy.parameter import Parameter


class MinimalParam(rclpy.node.Node):
    """Implement logic for dynamic parameters."""

    def __init__(self):
        """Set up stuff."""
        node_name = 'param_test'
        super().__init__(node_name)

        self.declare_parameter('float_param', 42.0)
        self.declare_parameter('string_param', 'string')
        self.declare_parameter('integer_param', 12)
        self.add_on_set_parameters_callback(self._params_cb)
        self.float_param = (
            self.get_parameter('float_param')
            .get_parameter_value()
            .double_value
        )
        self.string_param = (
            self.get_parameter('string_param')
            .get_parameter_value()
            .string_value
        )
        self.integer_param = (
            self.get_parameter('integer_param')
            .get_parameter_value()
            .integer_value
        )
        self.timer = self.create_timer(1, self.timer_callback)

    def _params_cb(self, updated_parameters) -> SetParametersResult:
        """Update parameters."""
        for parameter in updated_parameters:
            if (
                parameter.name == 'float_param'
                and parameter.type_ == Parameter.Type.DOUBLE
            ):
                self.float_param = parameter.value
            elif (
                parameter.name == 'string_param'
                and parameter.type_ == Parameter.Type.STRING
            ):
                self.string_param = parameter.value
            elif (
                parameter.name == 'integer_param'
                and parameter.type_ == Parameter.Type.INTEGER
            ):
                self.integer_param = parameter.value

        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Run the main logic."""
        self.get_logger().info(
            f' float_param = {self.float_param}'
            f', string_param = {self.string_param}'
            f', integer_param = {self.integer_param}'
        )


def main():
    """Implement the main logic."""
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
