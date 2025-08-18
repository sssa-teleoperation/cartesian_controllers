from setuptools import setup
from glob import glob
import os

package_name = 'cartesian_motion_controller_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/data', glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Test package for the Cartesian Motion Controller in velocity mode',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_publisher_sinusoidal = cartesian_motion_controller_test.test_publisher_sinusoidal:main',
            'test_publisher_gradino = cartesian_motion_controller_test.test_publisher_gradino:main',
            'xdot_plot_publisher = cartesian_motion_controller_test.xdot_plot_publisher:main',
            'single_axis_publisher = cartesian_motion_controller_test.single_axis_publisher:main',
            'neural_6dof_publisher = cartesian_motion_controller_test.neural_6dof_publisher:main',
            'marker_publisher = cartesian_motion_controller_test.marker_publisher:main',
            'trial_publisher = cartesian_motion_controller_test.trial_publisher:main',
            'actualvel_publisher = cartesian_motion_controller_test.actualvel_publisher:main',
            'marker_publisher2 = cartesian_motion_controller_test.marker_pubblisher2:main',
            'actualvel_publisher2 = cartesian_motion_controller_test.actualvel_publisher2:main',
            'test = cartesian_motion_controller_test.test:main',
            'test_publisher_sinusoidal6 = cartesian_motion_controller_test.test_publisher_sinusoidal6:main',
            'target_publisher = cartesian_motion_controller_test.target_publisher:main',
        ],
    },
)