from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'palletizer_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=['conveyor_boxes'],
    entry_points={
        'console_scripts': [
            'conveyor_boxes = conveyor_boxes:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*'),),
        (os.path.join('share', package_name, 'meshes', 'ur5e', 'visual'), glob('meshes/ur5e/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'ur5e', 'collision'), glob('meshes/ur5e/collision/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ELE208 group',
    maintainer_email='you@example.com',
    description='6-DOF palletizing robot from the project DH tables (no CAD meshes), Gazebo/ROS 2 sim.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
