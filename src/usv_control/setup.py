#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Python setup script for usv_control.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船控制包(usv_control)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import find_packages, setup

package_name = 'usv_control'

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
    description='Control algorithms and modules for Unmanned Surface Vehicles',
    long_description="""该包实现了USV系统的控制算法和模块，
包括路径规划、避障和自主导航等功能。
This package implements control algorithms and modules for Unmanned Surface Vehicle systems,
including path planning, obstacle avoidance, and autonomous navigation functionalities.""",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_avoidance_node = usv_control.usv_avoidance_node:main',
            'usv_command_node = usv_control.usv_command_node:main',
            'usv_control_node = usv_control.usv_control_node:main',
            'coord_transform_node = usv_control.coord_transform_node:main',
            'velocity_controller_node = usv_control.velocity_controller_node:main',
            'log_collector = usv_control.log_collector:main',
            'formation_follower_node = usv_control.formation_follower_node:main',
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'Control'],
    python_requires='>=3.6',
    url='https://github.com/chenhangwei/usv_workspace',
)