from setuptools import find_packages, setup

package_name = 'gs_gui'

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
    description='该包为USV（无人船）集群/单艇提供基于PyQt的图形化地面站界面，支持任务调度、状态监控、2D可视化等功能，便于操作与管理。This package provides a PyQt-based GUI ground station for USV (Unmanned Surface Vehicle) clusters or single units, supporting mission scheduling, status monitoring, 2D visualization, and convenient operation and management.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_gui_app = gs_gui.main_gui_app:main',

        ],
    },
)
