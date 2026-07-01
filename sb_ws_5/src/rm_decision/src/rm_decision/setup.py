from glob import glob
import os

from setuptools import setup

package_name = 'rm_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_nav_to_pose = rm_decision.test:main',
            'rmul_decision = rm_decision.rmul_decision:main',
            'rmuc_decision = rm_decision.rmuc_decision:main',
            'rmuc_decision_attack = rm_decision.rmuc_decision_attack:main',
            'rmuc_decision_defense = rm_decision.rmuc_decision_defense:main',
            'rmuc_decision_baolei = rm_decision.rmuc_decision_baolei:main',
            'rmuc_decision_qianshao = rm_decision.rmuc_decision_qianshao:main',
            'rmuc_decision_qianshao_center_first = rm_decision.rmuc_decision_qianshao_center_first:main',
            'rmuc_decision_qianshao_delayed_outpost = rm_decision.rmuc_decision_qianshao_delayed_outpost:main',
            'rmuc_decision_enermy_side_patrol = rm_decision.rmuc_decision_enermy_side_patrol:main',
            'rmuc_decision_enermy_side_patrol_2 = rm_decision.rmuc_decision_enermy_side_patrol_2:main',
            'tunnel_test_decision = rm_decision.tunnel_test_decision:main',
            'test = rm_decision.test:main',
            'rmuc_decision_qianshao_delayed_outpost_1 = rm_decision.rmuc_decision_qianshao_delayed_outpost_1:main',

        ],
    },
)
