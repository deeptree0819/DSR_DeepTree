from setuptools import find_packages, setup
from glob import glob

package_name = 'yolo_pick_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml') + glob('yolo_pick_demo/*.npy')
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deeptree',
    maintainer_email='deeptree@todo.todo',
    description='YOLO-based pick and place for Doosan M0609 with RealSense depth camera',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'yolo_pick = yolo_pick_demo.yolo_pick_node:main',
            'yolo_pick_moveit = yolo_pick_demo.yolo_pick_moveit_node:main',
            'yolo_pick_sort_moveit = yolo_pick_demo.yolo_pick_sort_moveit_node:main',
            'yolo_pick_box_moveit = yolo_pick_demo.yolo_pick_box_moveit_node:main',
        ],
    },
)
