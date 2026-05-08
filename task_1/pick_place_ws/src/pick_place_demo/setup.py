import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_place_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deeptree',
    maintainer_email='deeptree@todo.todo',
    description='Pick and Place 데모 — MoveIt + OnRobot 그리퍼 기반 액션 서버/클라이언트',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller  = pick_place_demo.robot_controller:main',
            'pick_place_server = pick_place_demo.pick_place_server:main',
            'pick_place_client = pick_place_demo.pick_place_client:main',
        ],
    },
)
