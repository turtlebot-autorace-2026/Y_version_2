"""Gazebo, everything in one command:

    ros2 launch autorace sim.launch.py

Starts autorace_gazebo (hakee's map, if it is installed / sourced; gazebo:=false if it already runs),
camera bridges (Gazebo Image -> the CompressedImage topics the real cameras publish), the senior's sign
detector, the lane node, mission_fsm and cmd_vel_mux, all on simulation time.

  /camera/image_raw        -> /camera   (lane camera)
  /front_camera/image_raw  -> /camera2  (front camera: signs, traffic light, barrier)

If the Gazebo robot has no front camera, no sign is ever seen (missions do not start) and the traffic
light cannot be seen: the FSM then starts by itself after traffic_light.no_camera_go_s (5 s here).
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


def _setup(context):
    share = get_package_share_directory(PKG)
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    python = arg('python').strip() or (VENV_PYTHON if os.path.exists(VENV_PYTHON) else '')
    prefix = {'prefix': python} if python else {}
    lane_params = arg('lane_params') or os.path.join(share, 'config', 'lane_params.yaml')
    mission_params = arg('mission_params') or os.path.join(share, 'config', 'mission_params.yaml')
    show = arg('show').lower() == 'true'
    sim = {'use_sim_time': True}

    gz = arg('gazebo').strip().lower()
    try:
        gz_share = get_package_share_directory('autorace_gazebo')
    except PackageNotFoundError:
        gz_share = None
    if gz == 'true' and gz_share is None:
        raise RuntimeError('gazebo:=true but autorace_gazebo is not installed / sourced')
    start_gazebo = gz_share is not None and gz in ('true', 'auto')

    actions = [LogInfo(msg='[autorace sim] gazebo=%s yolo python=%s start_mode=%s'
                       % (start_gazebo, python or 'python3 (script default)', arg('start_mode')))]
    if start_gazebo:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'autorace.launch.py')),
            launch_arguments={'x': arg('x'), 'y': arg('y'), 'yaw': arg('yaw'), 'gui': arg('gui'),
                              'referee': 'true'}.items()))
    elif gz == 'auto':
        actions.append(LogInfo(msg='[autorace sim] autorace_gazebo not found: start Gazebo yourself'))
    actions += [
        Node(package=PKG, executable='sim_camera_bridge', name='lane_camera_bridge', output='screen',
             parameters=[sim, {'in_topic': '/camera/image_raw', 'out_topic': '/camera'}]),
        Node(package=PKG, executable='sim_camera_bridge', name='front_camera_bridge', output='screen',
             parameters=[sim, {'in_topic': '/front_camera/image_raw', 'out_topic': '/camera2'}]),
        Node(package=PKG, executable='sign_detection', name='mission_detection_node', output='screen',
             parameters=[sim, {'show': show, 'device': arg('device')}], **prefix),
        Node(package=PKG, executable='lane_detection', name='lane_detection_ad', output='screen',
             parameters=[lane_params, sim, {
                 'start_enabled': False,
                 'cmd_vel_topic': '/cmd_vel_lane',
                 'show': show,
                 'device': arg('device'),
                 'v_max': float(arg('v_max')),
             }], **prefix),
        Node(package=PKG, executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
             parameters=[mission_params, sim]),
        Node(package=PKG, executable='mission_fsm', name='mission_fsm', output='screen',
             parameters=[mission_params, sim, {'start_mode': arg('start_mode'),
                                               'traffic_light.no_camera_go_s': 5.0}]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo', default_value='auto', description='include autorace_gazebo: auto | true | false'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('x', default_value='0.687', description='spawn (autorace_gazebo default)'),
        DeclareLaunchArgument('y', default_value='-1.999'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('start_mode', default_value='light', description='light | go'),
        DeclareLaunchArgument('v_max', default_value='0.18'),
        DeclareLaunchArgument('python', default_value='', description='python with ultralytics for the YOLO nodes'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('lane_params', default_value=''),
        DeclareLaunchArgument('mission_params', default_value=''),
        OpaqueFunction(function=_setup),
    ])
