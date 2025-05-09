from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'doorbusters'

setup(
    name=package_name,
    version='0.0.1',
#    package_dir={
#        "": "."
#    },
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools',
                      'numpy',
                      'pid_controller'],
    zip_safe=True,
    maintainer='tarun',
    maintainer_email='tarun@todo.todo',
    description='Doorbusters project to control swarms of robots through tight spaces',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
            'doorbuster_node = doorbusters.doorbusters_node:main',
            'publisher = doorbusters.odometry_publisher:main',
            'subscriber = doorbusters.drone_control_subscriber:main',
            'final_control = doorbusters.final_control:main',
            'target_publisher = doorbusters.target_publisher:main',
        ],
    },
)
