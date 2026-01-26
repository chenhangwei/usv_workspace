#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Python setup script for usv_drivers.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船驱动包(usv_drivers)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

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
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='Hardware drivers for Unmanned Surface Vehicle sensors and actuators',
    long_description="""该包提供了USV系统中各种传感器和执行器的ROS 2驱动，
包括激光雷达、超声波传感器、UWB定位模块和电机控制器等。
This package provides ROS 2 drivers for various sensors and actuators used in 
Unmanned Surface Vehicle systems, including LiDAR, ultrasonic sensors, UWB positioning 
modules, and motor controllers.""",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_uwb_node = usv_drivers.usv_uwb_node:main',
            'usv_laserscan_node = usv_drivers.usv_laserscan_node:main',
            'usv_ultrasonic_node = usv_drivers.usv_ultrasonic_node:main',
            'usv_su04_node = usv_drivers.usv_su04_node:main',
            'usv_ultrasonic_radar_node = usv_drivers.usv_ultrasonic_radar_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'Drivers'],
    python_requires='>=3.6',
    url='https://github.com/chenhangwei/usv_workspace',
)