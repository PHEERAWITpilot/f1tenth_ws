from setuptools import find_packages, setup

package_name = 'imu_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/imu_bridge_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f2',
    maintainer_email='your_email@example.com',
    description='Arduino IMU Bridge - publishes yaw data from Arduino MKRZero IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = imu_bridge.arduino_bridge_node:main',
        ],
    },
)

