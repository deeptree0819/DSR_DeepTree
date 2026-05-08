import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stt_robot_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deeptree',
    maintainer_email='deeptree@todo.todo',
    description='STT 음성 명령 기반 Pick & Place 데모',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stt_node           = stt_robot_demo.stt_node:main',
            'nlp_node           = stt_robot_demo.nlp_node:main',
            'stt_pick_and_place = stt_robot_demo.stt_pick_and_place:main',
            'tts_node           = stt_robot_demo.tts_node:main',
        ],
    },
)
