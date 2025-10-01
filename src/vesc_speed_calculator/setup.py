from setuptools import setup
import os
from glob import glob

package_name = 'vesc_speed_calculator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f2',
    maintainer_email='your_email@example.com',
    description='VESC Speed Calculator - converts ERPM to linear velocity',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vesc_speed_calculator = vesc_speed_calculator.vesc_speed_calculator_node:main',
        ],
    },
)

