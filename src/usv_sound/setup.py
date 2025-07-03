from setuptools import find_packages, setup

package_name = 'usv_sound'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/'+package_name +'/resource',['resource/moon101.wav']),
          ('share/'+package_name +'/resource',['resource/gaga101.wav']),
           ('share/'+package_name +'/resource',['resource/gaga102.wav']),
            ('share/'+package_name +'/resource',['resource/gaga103.wav']),
             ('share/'+package_name +'/resource',['resource/gaga104.wav']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='该包为USV（无人船）系统提供声音播放与语音提示功能，实现任务状态、警告等信息的音频播报，提升系统交互性和安全性。This package provides sound playback and voice prompt functions for USV (Unmanned Surface Vehicle) systems, enabling audio broadcasting of task status, warnings, and other information to enhance system interactivity and safety.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usv_sound_node = usv_sound.usv_sound_node:main',
  
        ],
    },
)
