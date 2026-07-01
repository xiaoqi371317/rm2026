from glob import glob
import os

from setuptools import setup

package_name = 'tunnel_mode_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='m19863105606@163.com',
    description='Route-aware follow-yaw mode manager for tunnel traversal.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tunnel_mode_manager_node = tunnel_mode_manager.tunnel_mode_manager_node:main',
        ],
    },
)
