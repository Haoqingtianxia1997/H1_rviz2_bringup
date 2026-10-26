from setuptools import setup
from glob import glob
import os

package_name = 'h1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ] + [
        (os.path.join('share', package_name, directory),
         [os.path.join(directory, name) for name in files])
        for base in ('model',)
        for directory, _, files in os.walk(base) if files
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='xuanhaosong',
    maintainer_email='',
    description='H1 bringup: lowstate->joint_states + robot_state_publisher + rviz',
    license='Apache-2.0',
    entry_points={},
)
