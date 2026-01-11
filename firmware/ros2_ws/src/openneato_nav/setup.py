import os
from glob import glob
from setuptools import setup

package_name = 'openneato_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installa tutti i file launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        # Installa tutti i file di configurazione (params)
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
	# Install the urdf descriptor of Neato
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenNeato Maintainer',
    maintainer_email='maintainer@openneato.org',
    description='Navigation configuration and launch files for OpenNeato',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
