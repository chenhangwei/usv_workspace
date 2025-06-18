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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='功能包用于实现无人水面艇(USV)的自定义动作接口,The usv_action package provides custom action interfaces for Unmanned Surface Vehicles (USV), supporting task scheduling and control based on ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_head_action_node = usv_action.usv_head_action_node:main',
        ],
    },
)
