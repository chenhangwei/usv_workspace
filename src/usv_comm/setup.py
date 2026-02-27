#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 Chen Hangwei
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
无人船通信包(usv_comm)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import find_packages, setup

package_name = 'usv_comm'

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
    description='Communication modules for Unmanned Surface Vehicles',
    long_description="""该包提供了USV系统的通信模块，
包括状态处理、数据交换和命令传输等功能。
This package provides communication modules for Unmanned Surface Vehicle systems,
including status processing, data exchange, and command transmission functionalities.""",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_status_node = usv_comm.usv_status_node:main',
            'auto_set_home_node = usv_comm.auto_set_home_node:main',
            'gps_to_local_node = usv_comm.gps_to_local_node:main',
            'mock_usv_data = usv_comm.mock_usv_data:main',
            'navigate_to_point_node = usv_comm.navigate_to_point_node:main',
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'Communication'],
    python_requires='>=3.8',
    url='https://github.com/chenhangwei/usv_workspace',
)
