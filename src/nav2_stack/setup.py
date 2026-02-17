from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'behavior_trees'), glob(os.path.join('behavior_trees', '*.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lexij',
    maintainer_email='lexi.j.hanlon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_follower = nav2_stack.path_follower:main',
            'pose_pub = nav2_stack.pose_pub:main'
        ],
    },
)
