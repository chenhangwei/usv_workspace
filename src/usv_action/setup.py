"""
无人船动作接口包(usv_action)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import find_packages, setup

package_name = 'usv_action'

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
    description='Custom action interfaces for Unmanned Surface Vehicles',
    long_description="""功能包用于实现无人水面艇(USV)的自定义动作接口，
支持基于ROS 2的任务调度和控制。
The usv_action package provides custom action interfaces for Unmanned Surface Vehicles (USV), 
supporting task scheduling and control based on ROS 2.""",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_head_action_node = usv_action.usv_head_action_node:main',
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'Action'],
    python_requires='>=3.6',
    url='https://github.com/chenhangwei/usv_workspace',
)