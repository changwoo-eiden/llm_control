from setuptools import setup
package_name = 'my_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/my_perception', ['scripts/yolo_v5_ros2_node']),  # ★ 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'yolo_v5_ros2_node = my_perception.yolo_v5_ros2_node:main',
        ],
    },
)
