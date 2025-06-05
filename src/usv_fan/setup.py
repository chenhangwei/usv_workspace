from setuptools import find_packages, setup

package_name = 'usv_fan'

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
    description='该包为USV（无人船）系统提供风扇控制节点，实现温度监测与自动风扇开关，保障设备安全运行。This package provides a fan control node for USV (Unmanned Surface Vehicle) systems, enabling temperature monitoring and automatic fan switching to ensure safe device operation.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_fan_node = usv_fan.usv_fan_node:main',
        ],
    },
)
