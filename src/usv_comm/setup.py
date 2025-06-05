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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='该包为USV（无人船）系统提供通信相关节点与功能，包括状态信息的组装与发布、自动设置home点等，支持多模块间的数据交互。This package provides communication-related nodes and functions for USV (Unmanned Surface Vehicle) systems, including status message assembly and publishing, auto home point setting, and supports data exchange between multiple modules.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_status_node = usv_comm.usv_status_node:main',
            'auto_set_home_node= usv_comm.auto_set_home_node:main'
        ],
    },
)
