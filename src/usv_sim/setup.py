import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'usv_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='PX4 + Gazebo 单载具独立 SITL 仿真与 RL 启动包',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
