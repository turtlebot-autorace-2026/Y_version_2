"""Real robot, everything in one command (ROS 2 Galactic):

    ros2 launch autorace robot.launch.py

Starts, in this order of data flow:
  turtlebot3_bringup robot.launch.py   /odom, /scan, takes /cmd_vel         (only on the robot, see bringup)
  camera_publisher   (senior camera_ros publisher)   /camera  lane camera, undistorted 640x480
  camera_publisher2  (senior camera_ros publisher2)  /camera2 front camera (/dev/video2)
  sign_detection     (senior mission_model, YOLO11n) /parking /left /right /stop /construction /intersection /tunnel
  lane_detection     (lane_detection_ad, YOLO26n-seg) /cmd_vel_lane
  mission_fsm        (missions_v1)                    which mission, lane inputs, /mission/drive, /cmd_vel_mission
  cmd_vel_mux        (missions_v1)                    /cmd_vel (only the selected source)

bringup / cameras: auto = start them only when this computer is the robot (/dev/ttyACM0, the OpenCR, exists).
On a laptop they are skipped: run `ros2 launch autorace robot.launch.py` on the robot too, or start them there.
YOLO nodes run with `python` (default: ~/autorace/yolo_venv/bin/python if it exists, else the normal python3),
the same rule as the senior's mission_model launch.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'autorace'
VENV_PYTHON = os.path.expanduser('~/autorace/yolo_venv/bin/python')
OPENCR = '/dev/ttyACM0'


def _flag(context, name, auto_value):
    v = LaunchConfiguration(name).perform(context).strip().lower()
    if v == 'auto':
        return auto_value
    if v in ('true', '1', 'yes'):
        return True
    if v in ('false', '0', 'no'):
        return False
    raise RuntimeError('%s must be auto, true or false (got %r)' % (name, v))


def _python(context):
    p = LaunchConfiguration('python').perform(context).strip()
    if p:
        return p
    return VENV_PYTHON if os.path.exists(VENV_PYTHON) else ''


def _setup(context):
    share = get_package_share_directory(PKG)
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    on_robot = os.path.exists(OPENCR)
    bringup = _flag(context, 'bringup', on_robot)
    cameras = _flag(context, 'cameras', on_robot)
    python = _python(context)
    prefix = {'prefix': python} if python else {}
    lane_params = arg('lane_params') or os.path.join(share, 'config', 'lane_params.yaml')
    mission_params = arg('mission_params') or os.path.join(share, 'config', 'mission_params.yaml')
    show = arg('show').lower() == 'true'

    actions = [LogInfo(msg='[autorace] on_robot=%s bringup=%s cameras=%s yolo python=%s start_mode=%s v_max=%s'
                       % (on_robot, bringup, cameras, python or 'python3 (script default)',
                          arg('start_mode'), arg('v_max')))]
    if bringup:
        try:
            tb3 = get_package_share_directory('turtlebot3_bringup')
        except PackageNotFoundError:
            raise RuntimeError('bringup:=true but turtlebot3_bringup is not installed / sourced')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot.launch.py'))))
    if cameras:
        actions += [
            Node(package=PKG, executable='camera_publisher', name='camera_publisher', output='screen'),
            Node(package=PKG, executable='camera_publisher2', name='camera_publisher2', output='screen'),
        ]
    if not (bringup and cameras):
        actions.append(LogInfo(msg='[autorace] not starting %s here: they must run on the robot'
                               % ' + '.join(n for n, on in (('turtlebot3 bringup', bringup), ('cameras', cameras))
                                            if not on)))
    actions += [
        Node(package=PKG, executable='sign_detection', name='mission_detection_node', output='screen',
             parameters=[{'show': show, 'device': arg('device')}], **prefix),
        Node(package=PKG, executable='lane_detection', name='lane_detection_ad', output='screen',
             parameters=[lane_params, {
                 'start_enabled': False,             # mission_fsm starts it (/lane_state)
                 'cmd_vel_topic': '/cmd_vel_lane',   # only cmd_vel_mux talks to the robot
                 'show': show,
                 'device': arg('device'),
                 'v_max': float(arg('v_max')),
             }], **prefix),
        Node(package=PKG, executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
             parameters=[mission_params]),
        Node(package=PKG, executable='mission_fsm', name='mission_fsm', output='screen',
             parameters=[mission_params, {'start_mode': arg('start_mode')}]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='auto', description='turtlebot3_bringup: auto | true | false'),
        DeclareLaunchArgument('cameras', default_value='auto', description='camera_ros publishers: auto | true | false'),
        DeclareLaunchArgument('start_mode', default_value='light', description='light = wait for green | go'),
        DeclareLaunchArgument('v_max', default_value='0.08', description='lane speed m/s'),
        DeclareLaunchArgument('python', default_value='', description='python with ultralytics for the YOLO nodes'),
        DeclareLaunchArgument('device', default_value='', description="YOLO device: '' auto, 'cpu', '0'"),
        DeclareLaunchArgument('show', default_value='false', description='cv2 windows (need a display)'),
        DeclareLaunchArgument('lane_params', default_value='', description='default: config/lane_params.yaml'),
        DeclareLaunchArgument('mission_params', default_value='', description='default: config/mission_params.yaml'),
        OpaqueFunction(function=_setup),
    ])
