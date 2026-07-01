from setuptools import setup

package_name = 'twist_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='A GUI for publishing Twist messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_publisher_gui = twist_gui.twist_publisher_gui:main',
            'if_follow_gui = twist_gui.if_follow_gui:main',
            'posture = twist_gui.posture:main',
        ],
    },
)
