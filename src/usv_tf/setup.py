#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Python setup script for usv_tf.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船坐标变换包(usv_tf)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import find_packages, setup

package_name = 'usv_tf'

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
    description='Coordinate transformation modules for Unmanned Surface Vehicles',
    long_description="""该包处理USV系统中的TF变换和坐标管理，
提供车辆中不同组件和参考帧之间 essential 的空间关系定义。
This package handles TF transformations and coordinate management for USV systems,
providing essential spatial relationship definitions between different components
and reference frames in the vehicle.""",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_laser_node = usv_tf.static_tf_laser_node:main',
            'odom_to_tf = usv_tf.odom_to_tf:main',
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'TF', 'Transform'],
    python_requires='>=3.6',
    url='https://github.com/chenhangwei/usv_workspace',
)