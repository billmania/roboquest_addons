#!/usr/bin/env python

"""
Demonstrate the APRIL tag and LiDAR functionality.

Executed via an ssh terminal session on the robot.
"""

import argparse
from sys import exit as sys_exit
from typing import List

import docker

VERSION = '1'


class AprilDemo(object):
    """Demonstrate APRIL tags and LiDAR."""

    def __init__(self):
        """Prepare the demonstration."""
        self._version = VERSION

        self._waypoints = []
        self._parsed_args = self._parse_args()
        self._setup_docker()

    def _parse_args(self) -> argparse.Namespace:
        """Get the arguments from the command line."""
        parser = argparse.ArgumentParser(
            prog='APRILdemo',
            description='Demonstrate APRIL tags with LiDAR',
            epilog=''
        )

        parser.add_argument(
            '--docker_url',
            dest='docker_url',
            default=None,
            type=str
        )

        return parser.parse_args()

    def _setup_docker(self):
        """Create a client connection with the docker server."""
        try:
            if self._parsed_args.docker_url:
                self._docker_client = docker.DockerClient(
                    base_url=self._parsed_args.docker_url
                )
            else:
                self._docker_client = docker.from_env()

        except docker.errors.APIError as e:
            print(
                '_setup_docker caught docker.errors.APIError:'
                f' {e}'
            )
            sys_exit(2)

    def _kill_containers(self):
        """Kill any running containers."""
        try:
            containers = self._docker_client.containers.list()

            print(
                'ID: image  name   status'
            )
            for container in containers:
                print(
                    f'{container.short_id}:'
                    f' {container.image.tags}'
                    f' {container.name}'
                    f' {container.status}'
                )

        except docker.errors.APIError as e:
            print(
                '_kill_containers caught docker.errors.APIError:'
                f' {e}'
            )
            sys_exit(1)

    def _confirm_images(self):
        """Ensure the required images exist."""
        pass

    def _collect_user_waypoints(self) -> List:
        """Gather the list of waypoints from the user."""
        pass

    def _calculate_domain_id(self) -> str:
        """Calculate the unique ROS_DOMAIN_ID."""
        return '99'

    def _start_rq_core(self):
        """Start the rq_core container."""
        pass

    def _start_rq_addons(self):
        """Start the rq_addons container."""
        pass

    def _run_demo(self, rq_addons):
        """Run the demonstration script."""
        pass

    def _wait_for_termination(self, rq_addons):
        """Wait for the demo to complete."""
        pass

    def main(self):
        """Execute the steps in the demo."""
        self._kill_containers()

        self._confirm_images()

        self._waypoints = self._collect_user_waypoints()

        self._ros_domain_id = self._calculate_domain_id()

        self._rq_core = self._start_rq_core()

        self._rq_addons = self._start_rq_addons()

        self._run_demo(self._rq_addons)

        self._wait_for_termination(self._rq_addons)

        self._kill_containers()


if __name__ == '__main__':
    AprilDemo().main()
