from setuptools import find_packages, setup

package_name = 'openneato_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='OpenNeato Maintainer',
    maintainer_email='maintainer@openneato.org',
    description='ROS 2 driver for Neato D7 Botvac using serial interface',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neato_driver = openneato_driver.driver_node:main',
            'mission_control = openneato_driver.mission_control:main',
            'docking_server = openneato_driver.docking_server:main',
        ],
    },
)
