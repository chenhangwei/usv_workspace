import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'usv_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='chenhangwei',
    maintainer_email='chenhangwei77777@hotmail.com',
    description='Minimal RL tooling for SITL-based USV avoidance training',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'preflight_check = usv_rl.preflight_check:main',
            'collect_scripted_dataset = usv_rl.collect_scripted_dataset:main',
            'train_behavior_cloning = usv_rl.train_behavior_cloning:main',
            'train_ppo_policy = usv_rl.train_ppo_policy:main',
            'train_mappo_policy = usv_rl.train_mappo_policy:main',
            'evaluate_policy = usv_rl.evaluate_policy:main',
            'evaluate_mappo_policy = usv_rl.evaluate_mappo_policy:main',
            'diagnose_mappo_loss_gradients = usv_rl.diagnose_mappo_loss_gradients:main',
            'collect_teacher_trajectories = usv_rl.collect_teacher_trajectories:main',
            'train_distill_policy = usv_rl.train_distill_policy:main',
            'policy_inference_node = usv_rl.policy_inference_node:main',
            'simple_sim_node = usv_rl.simple_sim:main',
            'multi_usv_sim_node = usv_rl.multi_usv_sim:main',
            'publish_nav_goal_once = usv_rl.publish_nav_goal_once:main',
            'publish_synthetic_neighbors = usv_rl.publish_synthetic_neighbors:main',
            'benchmark_online_policy = usv_rl.benchmark_online_policy:main',
            'compare_online_benchmark = usv_rl.compare_online_benchmark:main',
            'validate_online_candidate = usv_rl.validate_online_candidate:main',
            'smoke_recommended_sitl = usv_rl.smoke_recommended_sitl:main',
            'recommended_status = usv_rl.recommended_status:main',
        ],
    },
)