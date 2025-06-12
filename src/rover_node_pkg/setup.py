from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_node_pkg'



setup(

    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cam',
    maintainer_email='enderman9523@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = rover_node_pkg.python_serial_bridge:main',
            'command = rover_node_pkg.command_interface:main',
        ],
    },
)
