import os
from setuptools import setup

package_name = 'initialpose_waiter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Waits for initialpose before launching Nav2 lifecycle',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initialpose_waiter_node = initialpose_waiter.initialpose_waiter_node:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/initialpose_waiter']),

    ],
)

