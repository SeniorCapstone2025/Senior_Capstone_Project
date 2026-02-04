import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_controller = app.lidar_controller:main',
            'line_following = app.line_following:main',
            'object_tracking = app.object_tracking:main',
            'ar_app = app.ar_app:main',
            'hand_trajectory = app.hand_trajectory_node:main',
            'hand_gesture = app.hand_gesture:main',
            'inventory_fsm_node = app.inventory_fsm_node:main',
            'waypoint_navigator_node = app.waypoint_navigator_node:main',
            'qr_scanner_node = app.qr_scanner_node:main',
            'yolo_detector_node = app.yolo_detector_node:main',
        ],
    },
)
