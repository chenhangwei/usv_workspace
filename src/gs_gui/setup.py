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
无人船地面站GUI包(gs_gui)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import setup
import os
from glob import glob

package_name = 'gs_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        package_name: ['resource/*.qss', 'resource/*.ui', 'resource/*.xml'],
    },
    data_files=[
        # ① 安装ROS包索引标记文件
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # ② 安装 package.xml
        ('share/' + package_name, ['package.xml']),

        # ③ 安装 launch 文件（如果有）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # ④ 安装 resource 文件夹（包含静态标记文件、UI文件、样式表等）
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=[
        'setuptools',
        'PyQt5',
        'common_interfaces',
    ],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='PyQt-based GUI ground station for USV systems',
    long_description="""该包为USV（无人船）集群/单艇提供基于PyQt的图形化地面站界面，
支持任务调度、状态监控、2D可视化等功能，便于操作与管理。
This package provides a PyQt-based GUI ground station for USV (Unmanned Surface Vehicle)
clusters or single units, supporting mission scheduling, status monitoring, 2D visualization,
and convenient operation and management.""",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_gui_app = gs_gui.main_gui_app:main',
            'apf_neighbor_relay_node = gs_gui.apf_neighbor_relay_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Visualization',
    ],
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'GUI', 'PyQt5'],
    python_requires='>=3.8',
    url='https://github.com/chenhangwei/usv_workspace',
)
