"""Real robot, everything in one command (ROS 2 Galactic):

    ros2 launch autorace robot.launch.py

Starts, in this order of data flow:
  turtlebot3_bringup robot.launch.py   /odom, /scan, takes /cmd_vel         (only on the robot, see bringup)
  camera_publisher   (senior camera_ros publisher)   /camera  lane camera, undistorted 640x480
  camera_publisher2  (senior camera_ros publisher2)  /camera2 front camera
                     devices: config/camera_params.yaml (or lane_camera:= front_camera:=)
  sign_detection     (senior mission_model, YOLO11n) /parking /left /right /stop /construction /intersection /tunnel
  lane_detection     (lane_detection_ad, YOLO26n-seg) /cmd_vel_lane
  mission_fsm        (missions_v1)                    which mission, lane inputs, /mission/drive, /cmd_vel_mission
  cmd_vel_mux        (missions_v1)                    /cmd_vel (only the selected source)

bringup / cameras: auto = start them only when this computer is the robot (/dev/ttyACM0, the OpenCR, exists).
On a laptop they are skipped: run `ros2 launch autorace robot.launch.py` on the robot too, or start them there.
YOLO nodes run with the venv python (active venv, <ws>/.venv, <ws>/src/autorace/.venv, ~/autorace/yolo_venv,
in that order; python:= overrides). Without one they fail at `import ultralytics`.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'autorace'


def find_venv_python():
    '''Python with ultralytics for the two YOLO nodes. Order:
      1. the venv that is active in this shell ($VIRTUAL_ENV, i.e. `source .venv/bin/activate` was run)
      2. <workspace>/.venv             (workspace = two levels above this package's install prefix)
      3. <workspace>/src/autorace/.venv
      4. ~/autorace/yolo_venv          (the senior's location)
    '' = none found: the nodes run with the system python3 and fail at `import ultralytics`.'''
    candidates = []
    if os.environ.get('VIRTUAL_ENV'):
        candidates.append(os.path.join(os.environ['VIRTUAL_ENV'], 'bin', 'python'))
    try:
        ws = os.path.abspath(os.path.join(get_package_prefix(PKG), '..', '..'))
        candidates += [os.path.join(ws, '.venv', 'bin', 'python'),
                       os.path.join(ws, 'src', 'autorace', '.venv', 'bin', 'python')]
    except PackageNotFoundError:
        pass
    candidates.append(os.path.expanduser('~/autorace/yolo_venv/bin/python'))
    for c in candidates:
        if os.path.exists(c):
            return c
    return ''
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
    return find_venv_python()


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
    if not python:
        actions.append(LogInfo(msg='[autorace] WARNING: no venv with ultralytics found (.venv in the workspace, '
                                   'or source .venv/bin/activate first). The YOLO nodes will fail to import.'))
    if bringup:
        try:
            tb3 = get_package_share_directory('turtlebot3_bringup')
        except PackageNotFoundError:
            raise RuntimeError('bringup:=true but turtlebot3_bringup is not installed / sourced')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot.launch.py'))))
    if cameras:
        # devices from config/camera_params.yaml; lane_camera:= / front_camera:= override for one run
        camera_params = arg('camera_params') or os.path.join(share, 'config', 'camera_params.yaml')
        lane_cam, front_cam = arg('lane_camera').strip(), arg('front_camera').strip()
        actions += [
            Node(package=PKG, executable='camera_publisher', name='camera_publisher', output='screen',
                 parameters=[camera_params] + ([{'device': lane_cam}] if lane_cam else [])),
            Node(package=PKG, executable='camera_publisher2', name='camera_publisher2', output='screen',
                 parameters=[camera_params] + ([{'device': front_cam}] if front_cam else [])),
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
             parameters=[mission_params, {'start_mode': arg('start_mode')}]
             + ([{'intersection_on_turn_sign': True}] if _flag(context, 'intersection_on_turn_sign', False) else [])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='auto', description='turtlebot3_bringup: auto | true | false'),
        DeclareLaunchArgument('cameras', default_value='auto', description='camera_ros publishers: auto | true | false'),
        DeclareLaunchArgument('start_mode', default_value='light', description='light = wait for green | go'),
        DeclareLaunchArgument('intersection_on_turn_sign', default_value='false',
                              description='true: /left /right start the intersection (practice map without '
                                          'the triangle sign; never on the course)'),
        DeclareLaunchArgument('v_max', default_value='0.18', description='lane speed on straights m/s (bends slow down by themselves)'),
        DeclareLaunchArgument('python', default_value='', description='python with ultralytics for the YOLO nodes'),
        DeclareLaunchArgument('device', default_value='', description="YOLO device: '' auto, 'cpu', '0'"),
        DeclareLaunchArgument('show', default_value='false', description='cv2 windows (need a display)'),
        DeclareLaunchArgument('lane_params', default_value='', description='default: config/lane_params.yaml'),
        DeclareLaunchArgument('mission_params', default_value='', description='default: config/mission_params.yaml'),
        DeclareLaunchArgument('camera_params', default_value='', description='default: config/camera_params.yaml'),
        DeclareLaunchArgument('lane_camera', default_value='',
                              description='lane camera device for this run, e.g. /dev/v4l/by-path/...-video-index0'),
        DeclareLaunchArgument('front_camera', default_value='', description='front camera device for this run'),
        OpaqueFunction(function=_setup),
    ])
