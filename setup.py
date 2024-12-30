import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'roboquest_addons'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.yml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bill Mania',
    maintainer_email='bill@manialabs.us',
    description='RoboQuest components outside the base functionality',
    license='Proprietary',
)
