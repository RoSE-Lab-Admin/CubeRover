from setuptools import find_packages, setup

package_name = 'rover_groundstation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cam',
    maintainer_email='enderman9523@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cameraviewer = rover_groundstation.image_sub:main',
            'motorbag = rover_groundstation.motor_bag:main',
            'univbag = rover_groundstation.universal_bag:main',
            'console = rover_groundstation.test_console:main',
        ],
    },
)
