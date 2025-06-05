from setuptools import find_packages, setup

package_name = 'usv_tf'

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
    description='该包为USV（无人船）系统提供坐标变换与TF广播功能，实现多坐标系（如地图、船体、传感器等）之间的实时转换与关联，支持导航与多传感器融合。This package provides coordinate transformation and TF broadcasting functions for USV (Unmanned Surface Vehicle) systems, enabling real-time conversion and association between multiple coordinate frames (such as map, base, and sensors), supporting navigation and multi-sensor fusion.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_laser_node = usv_tf.static_tf_laser_node:main',
            'odom_to_tf = usv_tf.odom_to_tf:main',
        ],
    },
)
