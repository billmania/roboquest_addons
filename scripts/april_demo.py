#!/usr/bin/env python3

"""
Demonstrate the APRIL tag and LiDAR functionality.

Executed via an ssh terminal session on the robot.
"""

import argparse
import logging
from subprocess import call as run_script
from sys import exit as sys_exit
from typing import List

import docker

from requests.exceptions import ConnectionError as URLTimeoutError

VERSION = '1'
TIMEOUT_DEFAULT = 120.0
IMAGES = ['rq_core_25.1rc3', 'rq_addons_4rc2']
SERIAL_NUMBER_FILE = '/sys/firmware/devicetree/base/serial-number'
WAYPOINTS_FILE = '/opt/persist/tags/waypoints.txt'
NULL_CHAR = '\0'
BUILD_IMAGES_URL = (
    'https://github.com/billmania/roboquest_addons'
    '/raw/refs/heads/main/scripts/build_images.sh'
)


class AprilDemo(object):
    """Demonstrate APRIL tags and LiDAR."""

    def __init__(self):
        """Prepare the demonstration."""
        logging.basicConfig(
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.DEBUG
        )
        logging.info(f'april_demo.py version {VERSION} started')

        self._waypoints = []
        self._parsed_args = self._parse_args()
        self._setup_docker()

    def _parse_args(self) -> argparse.Namespace:
        """Get the arguments from the command line."""
        logging.debug(
            '_parse_args called'
        )
        parser = argparse.ArgumentParser(
            prog='APRILdemo',
            description='Demonstrate APRIL tags with LiDAR',
            epilog=''
        )

        parser.add_argument(
            '--docker_url',
            dest='docker_url',
            default=None,
            help='URL to connect to the docker server',
            type=str
        )
        parser.add_argument(
            '--timeout',
            dest='timeout',
            default=TIMEOUT_DEFAULT,
            help='seconds to wait for rq_addons to terminate',
            type=float
        )
        parser.add_argument(
            '--waypoints',
            dest='waypoints',
            default='',
            help=(
                'a comma-separated, ordered list of'
                ' APRIL tag names to follow'
            ),
            type=str
        )
        parser.add_argument(
            '--ros_domain_id',
            dest='ros_domain_id',
            default='',
            help='Two digit ROS_DOMAIN_ID of the robot',
            type=str
        )
        parser.add_argument(
            '--images',
            dest='images',
            default='',
            help='a comma-separated list of the image names',
            type=str
        )

        return parser.parse_args()

    def _setup_docker(self):
        """Create a client connection with the docker server."""
        logging.debug(
            '_setup_docker called'
        )
        try:
            if self._parsed_args.docker_url:
                self._docker_client = docker.DockerClient(
                    base_url=self._parsed_args.docker_url
                )
            else:
                self._docker_client = docker.from_env()

        except docker.errors.APIError as e:
            logging.error(
                '_setup_docker caught docker.errors.APIError:'
                f' {e}'
            )
            sys_exit(2)

        except docker.errors.DockerException:
            logging.error(
                'failed to connect to docker server. check --docker_url'
            )
            sys_exit(3)

    def _kill_containers(self):
        """Kill any running Roboquest containers."""
        logging.debug(
            '_kill_containers called'
        )
        containers = self._docker_client.containers.list()

        for container in containers:
            try:
                if container.name.find('rq_') == 0:
                    logging.warning(
                        f' Killing {container.short_id}'
                    )
                    container.kill()

            except docker.errors.APIError as e:
                logging.error(
                    '_kill_containers caught docker.errors.APIError:'
                    f' {e}'
                )

    def _build_images(self) -> bool:
        """Build the docker images.

        Retrieve the latest version of the build_images.sh script.
        Execute it.
        """
        logging.debug(
            '_build_images called'
        )
        run_script([
            '/usr/bin/wget',
            '-O',
            '/tmp/build_images.sh',
            BUILD_IMAGES_URL
        ])

        run_script([
            '/bin/bash',
            '/tmp/build_images.sh'
        ])

    def _check_images(self, image_list: List = []):
        """Ensure the required images exist.

        If the two images already exist locally, they won't be rebuilt.
        Otherwise, the _build_images() method will be called.
        """
        logging.debug(
            '_check_images called'
        )
        image_missing = False
        for image_name in image_list:
            try:
                _ = self._docker_client.images.get(image_name + ':latest')

            except docker.errors.ImageNotFound:
                image_missing = True
                logging.warn(
                    '_check_images:'
                    f' {image_name} not found'
                )

        if image_missing:
            self._build_images()

    def _parse_waypoints(self, waypoints: List) -> List:
        """Parse the list of waypoints.

        Extract the waypoints from the command line argument
        and use them to overwrite the content of the waypoints
        file, so navigator.py can read them.
        """
        logging.debug(
            '_parse_waypoints called:'
            f' {waypoints}'
        )
        with open(WAYPOINTS_FILE, 'w', encoding='ascii') as f:
            for waypoint in waypoints.replace(' ', '').split(','):
                if waypoint != '':
                    f.write(waypoint + '\n')

    def _calculate_domain_id(self) -> str:
        """Calculate the unique ROS_DOMAIN_ID.

        Calculate an integer, between 1 and 101 inclusive, which is
        unique across the universe of Raspberry Pi units. Return
        the ID as a string.
        """
        logging.debug(
            '_calculate_domain_id called'
        )
        try:
            with open(SERIAL_NUMBER_FILE, 'r') as serial_file:
                cpuserial_raw = serial_file.read()

        except FileNotFoundError:
            logging.error(
                '_calculate_domain_id:'
                f' {SERIAL_NUMBER_FILE} does not exist.'
                ' Use --ros_domain_id.'
            )
            sys_exit(6)

        try:
            self._cpuserial = ''.join(filter(
                lambda character: character != NULL_CHAR,
                cpuserial_raw
            ))

            unique_id = (
                int(self._cpuserial, base=16)
                % 100) + 1
            return f'{unique_id}'

        except Exception:
            return '99'

    def _start_rq_core(self) -> docker.models.containers.Container:
        """Start the rq_core container."""
        logging.debug(
            '_start_rq_core called'
        )
        IMAGE_NAME = 'rq_core_25.1rc3'
        RQ_CORE = {
            'image_name': IMAGE_NAME,
            'devices': ['/dev/gpiomem:/dev/gpiomem:rwm',
                        '/dev/i2c-1:/dev/i2c-1:rwm',
                        '/dev/i2c-6:/dev/i2c-6:rwm',
                        '/dev/ttyAMA3:/dev/ttyAMA1:rwm'],
            'volumes': ['/dev/shm:/dev/shm',
                        '/var/run/dbus:/var/run/dbus',
                        '/run/udev:/run/udev:ro',
                        (
                            '/opt/persist'
                            ':'
                            '/usr/src/ros2ws/install'
                            '/roboquest_core/share/roboquest_core/persist'
                        ),
                        'ros_logs:/root/.ros/log']
        }

        image_name = RQ_CORE['image_name']
        try:
            rq_core = self._docker_client.containers.run(
                image_name,
                name='rq_core',
                detach=True,
                environment=['ROS_DOMAIN_ID='+self._ros_domain_id],
                privileged=True,
                auto_remove=True,
                devices=RQ_CORE['devices'],
                ipc_mode='host',
                network_mode='host',
                volumes=RQ_CORE['volumes']
            )
            rq_core.reload()
            logging.debug(
                '_start_rq_core:'
                f' started {rq_core.short_id}'
                f' {rq_core.name}'
                f' {rq_core.status}'
            )

        except docker.errors.ImageNotFound:
            logging.error(
                f'_start_rq_core: image {IMAGE_NAME} not found'
            )
            sys_exit(7)

        return rq_core

    def _start_rq_addons(self) -> docker.models.containers.Container:
        """Start the rq_addons container."""
        logging.debug(
            '_start_rq_addons called'
        )
        IMAGE_NAME = 'rq_addons_4rc2'
        RQ_ADDONS = {
            'image_name': IMAGE_NAME,
            'devices': [
                '/dev/i2c-6:/dev/i2c-6',
                '/dev/ttyUSB0:/dev/ttyUSB0'
            ],
            'volumes': [
                '/dev/shm:/dev/shm',
                '/var/run/dbus:/var/run/dbus',
                '/run/udev:/run/udev',
                (
                    '/opt/persist'
                    ':'
                    '/usr/src/ros2ws/install'
                    '/roboquest_addons/share/roboquest_addons/persist'
                ),
                'ros_logs:/root/.ros/log'
            ]
        }

        image_name = RQ_ADDONS['image_name']
        try:
            rq_addons = self._docker_client.containers.run(
                image_name,
                name='rq_addons',
                detach=True,
                tty=True,
                stdin_open=False,
                environment=['ROS_DOMAIN_ID='+self._ros_domain_id],
                privileged=True,
                auto_remove=False,
                remove=True,
                devices=RQ_ADDONS['devices'],
                ipc_mode='host',
                network_mode='host',
                volumes=RQ_ADDONS['volumes'],
                command=(
                    '/usr/src/ros2ws'
                    '/src/roboquest_addons/scripts/autonomy_demo.sh'
                )
            )

            rq_addons.reload()
            logging.debug(
                f'started {rq_addons.short_id}'
                f' {rq_addons.name}'
                f' {rq_addons.status}'
            )

        except docker.errors.ImageNotFound:
            logging.error(
                f'_start_rq_addons: image {IMAGE_NAME} not found'
            )
            sys_exit(7)

        return rq_addons

    def _wait_for_termination(
         self,
         rq_addons: docker.models.containers.Container
    ):
        """Wait for the demo to complete."""
        logging.info(
            '_wait_for_termination'
            f' timeout {self._parsed_args.timeout} seconds'
        )
        rq_addons.reload()
        logging.debug(
            f'started {rq_addons.short_id}'
            f' {rq_addons.name}'
            f' {rq_addons.status}'
        )
        try:
            exit_info = rq_addons.wait(timeout=self._parsed_args.timeout)
            logging.debug(
                f"_wait_for_termination exit status: {exit_info['StatusCode']}"
            )

        except URLTimeoutError:
            logging.warning(
                '_wait_for_termination: time out waiting for terminate.'
            )
            logging.warning(
                '_wait_for_termination: killing it now.'
            )
            rq_addons.kill()

    def main(self):
        """Execute the steps in the demo."""
        logging.debug(
            'main called'
        )
        self._kill_containers()

        self._check_images(IMAGES)

        self._parse_waypoints(self._parsed_args.waypoints)

        if self._parsed_args.ros_domain_id != '':
            self._ros_domain_id = self._parsed_args.ros_domain_id
        else:
            self._ros_domain_id = self._calculate_domain_id()
        logging.info(
            f'ROS_DOMAIN_ID: {self._ros_domain_id}'
        )

        self._rq_core = self._start_rq_core()

        self._rq_addons = self._start_rq_addons()

        for output_chunk in self._rq_addons.logs(stream=True, follow=True):
            print(
                output_chunk.decode(errors='replace'),
                end='',
                flush=True
            )

        self._wait_for_termination(self._rq_addons)

        self._kill_containers()


if __name__ == '__main__':
    AprilDemo().main()
