from setuptools import setup, find_packages

package_name = 'dwa_local_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changwoo',
    maintainer_email='you@example.com',
    description='DWA Local Planner Example',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dwa_local_planner_node = dwa_local_planner.dwa_local_planner_node:main'
        ],
    },
)

