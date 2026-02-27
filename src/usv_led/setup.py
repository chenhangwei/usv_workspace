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
无人船LED控制包(usv_led)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import find_packages, setup

package_name = 'usv_led'

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
    description='LED control module for Unmanned Surface Vehicles',
    long_description="""该包提供了USV平台LED指示灯的控制接口，
支持状态指示、通信信号和各种操作模式的视觉反馈。
This package provides interfaces for controlling LED indicators on USV platforms,
supporting status indication, communication signaling, and visual feedback for
various operational modes.""",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_led_node = usv_led.usv_led_node:main',
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'LED Control'],
    python_requires='>=3.8',
    url='https://github.com/chenhangwei/usv_workspace',
)