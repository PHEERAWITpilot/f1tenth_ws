from setuptools import setup

package_name = 'nav2_waypoint_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Send waypoints to the Nav2 waypoint follower',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_waypoints = nav2_waypoint_launch.send_waypoints:main',
        ],
    },
)

