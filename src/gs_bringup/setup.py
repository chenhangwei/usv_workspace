"""
地面站启动包(gs_bringup)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

import os
from setuptools import find_packages, setup

package_name = 'gs_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/gs_launch.py']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='ROS 2 package for USV ground station launch management',
    long_description="""该包用于USV（无人船）系统的启动与一键集成管理，
包含多节点/多模块的统一启动脚本和配置，便于集群或单艇的快速部署和调试。
This package provides bringup and integrated launch management for USV 
(Unmanned Surface Vehicle) systems, including unified launch scripts and 
configurations for multi-node/multi-module deployment, enabling fast 
deployment and debugging for single or multiple USVs.""",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'Ground Station'],
    python_requires='>=3.6',
    url='https://github.com/chenhangwei/usv_workspace',
)