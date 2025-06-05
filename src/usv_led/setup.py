from setuptools import find_packages, setup

package_name = 'usv_led'

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
    description='该包为USV（无人船）系统提供LED灯控制节点，实现状态指示、任务反馈等功能，便于设备运行状态的可视化。This package provides LED control nodes for USV (Unmanned Surface Vehicle) systems, enabling status indication, task feedback, and visualizing device operation states.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_led_node = usv_led.usv_led_node:main',
        ],
    },
)
