from setuptools import find_packages, setup

package_name = 'usv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name, ['usv_control/local_planner_node.py']), 

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
            'local_planner_node = usv_control.local_planner_node:main'
        ],
    },
)
