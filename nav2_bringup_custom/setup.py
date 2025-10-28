from setuptools import setup

package_name = 'nav2_bringup_custom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),  # <-- This line ensures package.xml is installed
    ('share/' + package_name + '/launch', ['launch/nav2_bringup.launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Custom bringup for Nav2 system including prerequisites.',
    license='Apache License 2.0',
)

