"""
无人球声音控制包(usv_sound)的Python打包配置文件
该文件定义了ROS 2包的元数据和安装要求
"""

from setuptools import find_packages, setup

package_name = 'usv_sound'

# 获取所有资源文件
def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # 添加声音资源文件
    sound_files = [
        'moon101.wav',
        'gaga101.wav',
        'gaga102.wav',
        'gaga103.wav',
        'gaga104.wav'
    ]
    
    for sound_file in sound_files:
        data_files.append(('share/' + package_name + '/resource', 
                          ['resource/' + sound_file]))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='Sound control module for Unmanned Surface Vehicles',
    long_description="""该包提供了USV平台音频系统的接口，
支持警报信号、通信音调和各种状态事件的声学反馈。
This package provides interfaces for audio systems on USV platforms,
supporting alert signals, communication tones, and acoustic feedback
for various operational states and events.""",
    long_description_content_type='text/plain',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_sound_node = usv_sound.usv_sound_node:main',
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
    keywords=['ROS', 'ROS2', 'USV', 'Unmanned Surface Vehicle', 'Sound Control'],
    python_requires='>=3.10',
    url='https://github.com/chenhangwei/usv_workspace',
)
