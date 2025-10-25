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
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_gui_app = gs_gui.main_gui_app:main',
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
    python_requires='>=3.6',
    url='https://github.com/chenhangwei/usv_workspace',
)
