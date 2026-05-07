#!/usr/bin/env python
"""Test implementation of ROS2 dynamic parameters.

Demonstrate how a node can define parameters itself at run-time,
without requiring them to be defined at startup with a launch
file or with "ros2 run". The parameters can then be modified at
run-time using "ros2 param set /param_test".
"""
from typing import List, Tuple, Union

from rcl_interfaces.msg import SetParametersResult

import rclpy
import rclpy.node

declarations = [
    ('float_param', 42.0),
    ('string_param', 'string'),
    ('integer_param', 1)
]


class MinimalParam(rclpy.node.Node):
    """Implement logic for dynamic parameters."""

    def __init__(self):
        """Set up stuff."""
        node_name = 'param_test'
        super().__init__(node_name)

        self._parameters = self._setup_parameters(declarations)
        self.add_on_set_parameters_callback(self._params_cb)
        self.timer = self.create_timer(1, self.timer_callback)

    def _setup_parameters(
         self,
         declarations: List[Tuple[str, Union[float, str, int]]]) -> dict:
        """Set up the parameters.

        Receive the parameter declarations as a list of tuples containing
        the name of the parameter and its default value.
        Return a dictionary with each parameter name as the key
        and the parameter's current value.
        """
        parameters = {}

        for declaration in declarations:
            param_name = declaration[0]
            param_value = declaration[1]
            self.declare_parameter(param_name, param_value)

            parameters[param_name] = self.get_parameter(param_name)

        return parameters

    def _params_cb(self, updated_parameters) -> SetParametersResult:
        """Update parameters.

        updated_parameters seems to always be a list of one parameter,
        whether calling "ros2 param set" with a single parameter or
        "ros2 param load" with a YAML file containing multiple
        parameters.
        """
        self.get_logger().info(
            '_params_cb:'
            f' {len(updated_parameters)} updated parameters'
        )

        for parameter in updated_parameters:
            self._parameters[parameter.name] = parameter

        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Run the main logic."""
        for key in self._parameters:
            self.get_logger().info(
                f' {key} = {self._parameters[key].value}'
            )


def main():
    """Implement the main logic."""
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
