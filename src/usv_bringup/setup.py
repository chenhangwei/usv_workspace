from setuptools import find_packages, setup

package_name = 'usv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name +'/config',['config/usv_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/usv_launch.py']),  # 添加 launch 文件夹
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='该包用于USV（无人船）系统的启动与集成管理，提供多节点/多模块的一键启动脚本和配置，方便单艇或集群的快速部署和调试。This package provides bringup and integrated launch management for USV (Unmanned Surface Vehicle) systems, including unified launch scripts and configurations for multi-node/multi-module deployment, enabling fast deployment and debugging for single or multiple USVs.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
