from setuptools import setup

package_name = 'reference_data_gui'

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
    maintainer='Jiang_MG',
    maintainer_email='@email.com',
    description='A GUI for publishing ReferenceDataMini messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reference_data_gui = reference_data_gui.reference_data_mini_gui:main',
        ],
    },
)
