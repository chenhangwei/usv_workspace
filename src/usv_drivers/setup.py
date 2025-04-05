from setuptools import find_packages, setup

package_name = 'usv_drivers'

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
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [              
            'usv_ultrasonic_node=usv_drivers.usv_ultrasonic_node:main',
            'usv_laserscan_node=usv_drivers.usv_laserscan_node:main',
            'usv_uwb_node=usv_drivers.usv_uwb_node:main',
        ],
    },
)
