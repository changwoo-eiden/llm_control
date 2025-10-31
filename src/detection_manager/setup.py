# ~/ros2_ws/src/detection_manager/setup.py
from setuptools import setup

package_name = 'detection_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/RunDetection.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changwoo',
    maintainer_email='you@example.com',
    description='YOLO detection service for GPT controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_manager_node = detection_manager.detection_manager_node:main',
        ],
    },
)
