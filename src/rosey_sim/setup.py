from setuptools import find_packages, setup

package_name = 'rosey_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rosey_gz.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/world.world']),
        ('share/' + package_name + '/config', ['config/ros_gz_bridge.yaml']),

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
        ],
    },
)
