from setuptools import find_packages, setup

package_name = 'usv_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','px4_msgs'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='The usv_station package provides ROS 2 nodes for managing onboard computer programs of Unmanned Surface Vehicles (USVs). It supports control, pose estimation, and status monitoring for a cluster of USVs, enabling formation control and coordination in multi-vehicle scenarios.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_interface_node = usv_station.px4_interface_node:main',
            'usv_control_node = usv_station.usv_control_node:main',
        ],
    },
)
