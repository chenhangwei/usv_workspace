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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='该包为USV（无人船）系统提供控制相关节点与功能，包括目标点、速度、避障等控制指令的处理与发布，支持多种控制模式和任务需求。This package provides control-related nodes and functions for USV (Unmanned Surface Vehicle) systems, including processing and publishing of setpoints, velocity, and obstacle avoidance commands, supporting various control modes and mission requirements.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_control_node = usv_control.usv_control_node:main',
            'usv_avoidance_node = usv_control.usv_avoidance_node:main',
            'usv_command_node = usv_control.usv_command_node:main',
        ],
    },
)
