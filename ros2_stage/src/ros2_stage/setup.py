from glob import glob
from setuptools import setup

package_name = 'ros2_stage'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabeb',
    maintainer_email='rabeb@example.com',
    description='Kinematic simulation of a differential drive robot with trailer.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematic_trailer_node = ros2_stage.kinematic_trailer_node:main',
            'cmd_vel_demo_node = ros2_stage.cmd_vel_demo_node:main',
        ],
    },
)
