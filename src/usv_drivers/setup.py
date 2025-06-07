from setuptools import find_packages, setup

package_name = 'usv_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='该包为USV（无人船）系统提供各类硬件驱动节点，包括激光雷达、超声波、UWB等传感器的接入与数据发布，支持多种底层设备的集成。This package provides various hardware driver nodes for USV (Unmanned Surface Vehicle) systems, including integration and data publishing for sensors such as LiDAR, ultrasonic, and UWB, supporting the connection of multiple underlying devices.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [              
            'usv_ultrasonic_node=usv_drivers.usv_ultrasonic_node:main',
            'usv_laserscan_node=usv_drivers.usv_laserscan_node:main',
            'usv_uwb_node=usv_drivers.usv_uwb_node:main',
            'usv_su04_node=usv_drivers.usv_su04_node:main',
            'usv_ultrasonic_radar_node=usv_drivers.usv_ultrasonic_radar_node:main',
        ],
    },
)
