import glob

from setuptools import find_packages, setup

package_name = 'autorace'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml') + glob.glob('config/*.pkl')),
        ('share/' + package_name + '/models', glob.glob('models/*.pt') + glob.glob('models/*.jpg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlebot-autorace-2026',
    maintainer_email='eureka01020304@gmail.com',
    description='TurtleBot3 AutoRace 2026 in one package: cameras, YOLO lane + sign models, lane following, '
                'mission FSM and cmd_vel mux',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # senior's camera_ros
            'camera_publisher = autorace.camera.publisher:main',
            'camera_publisher2 = autorace.camera.publisher2:main',
            'camera_viewer = autorace.camera.viewer:main',
            # senior's mission_model (sign detection)
            'sign_detection = autorace.sign.detection:main',
            # lane_detection_ad
            'lane_detection = autorace.lane.detection:main',
            'sim_camera_bridge = autorace.lane.sim_camera_bridge:main',
            'section_judge = autorace.lane.section_judge:main',
            # missions_v1
            'mission_fsm = autorace.mission.mission_fsm:main',
            'cmd_vel_mux = autorace.mission.cmd_vel_mux:main',
        ],
    },
)
