#!/usr/bin/env python3
"""Read-only snapshot for matching the existing Gazebo setup to official FUEL."""
import argparse
import datetime
import hashlib
import io
import json
from pathlib import Path
import socket
import subprocess
import threading
import zipfile


def launch_files(home):
    """Only the known hover-fix directory; never scan the user's home."""
    root = home / 'rotors_ws' / 'rotors_vi_hover_fix'
    result = {}
    total = 0
    if not root.is_dir():
        return result, ['Hover launch directory not found: ' + str(root)]
    warnings = []
    for path in sorted(root.rglob('*')):
        if (path.is_symlink() or not path.is_file() or
                path.suffix.lower() not in ('.launch', '.yaml', '.yml', '.xacro', '.urdf', '.world')):
            continue
        if len(result) >= 60 or path.stat().st_size > 1000000:
            warnings.append('Skipped oversized or excess file: ' + str(path.relative_to(root)))
            continue
        data = path.read_bytes()
        if b'\x00' in data or total + len(data) > 3000000:
            warnings.append('Skipped non-text or excess file: ' + str(path.relative_to(root)))
            continue
        data.decode('utf-8')
        result['hover_launch/' + path.relative_to(root).as_posix()] = data
        total += len(data)
    return result, warnings


def capture_ros(mav, timeout):
    import rospy
    import rosnode
    import rosgraph
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    import genpy

    if not rosgraph.is_master_online():
        raise RuntimeError('ROS master is not running; keep the working Gazebo hover simulation open.')
    rospy.init_node('fuel_official_inputs', anonymous=True, disable_signals=True)
    prefix = '/' + mav
    params = {}
    for key in ('/use_sim_time', '/run_id', prefix + '/lee_position_controller_node',
                '/fuel_exploration/planner', '/fuel_exploration/sensor_bridge',
                '/fuel_navigation/planner', '/fuel_navigation/sensor_bridge'):
        params[key] = rospy.get_param(key, None)
    urdf = rospy.get_param(prefix + '/robot_description', '')
    if not isinstance(urdf, str) or '<robot' not in urdf:
        raise RuntimeError('Running robot description is missing: ' + prefix + '/robot_description')
    topics = {
        'odometry': (prefix + '/odometry_sensor1/odometry', Odometry),
        'camera_info': (prefix + '/vi_sensor/camera_depth/depth/camera_info', CameraInfo),
        'depth_metadata': (prefix + '/vi_sensor/camera_depth/depth/disparity', Image),
    }
    events = {key: threading.Event() for key in topics}
    messages = {}
    subscribers = []

    def callback(msg, key):
        if not events[key].is_set():
            if key == 'depth_metadata':
                messages[key] = json.dumps({
                    'frame_id': msg.header.frame_id,
                    'stamp_secs': msg.header.stamp.secs,
                    'stamp_nsecs': msg.header.stamp.nsecs,
                    'width': msg.width, 'height': msg.height,
                    'encoding': msg.encoding, 'step': msg.step,
                    'is_bigendian': msg.is_bigendian, 'data_bytes': len(msg.data),
                }, indent=2)
            else:
                stream = io.StringIO()
                stream.write(genpy.message.strify_message(msg))
                messages[key] = stream.getvalue()
            events[key].set()

    for key, (topic, msg_type) in topics.items():
        subscribers.append(rospy.Subscriber(topic, msg_type, callback, callback_args=key, queue_size=1))
    # Wait once for the pair using wall time, including when simulated time is paused.
    deadline = __import__('time').monotonic() + timeout
    for event in events.values():
        event.wait(max(0.0, deadline - __import__('time').monotonic()))
    for sub in subscribers:
        sub.unregister()
    summary = {
        'nodes': rosnode.get_node_names(),
        'parameters': params,
        'topics': {key: value[0] for key, value in topics.items()},
        'missing_messages': [key for key in topics if key not in messages],
        'published_topics': rospy.get_published_topics(),
        'robot_description_sha256': hashlib.sha256(urdf.encode()).hexdigest(),
        'parameter_note': 'Parameter server contents can outlive their launch. Check node list and capture time; these are not proof a node is currently running.',
    }
    source_paths = {}
    for package in ('rotors_description', 'rotors_gazebo', 'fuel_gazebo_mapping',
                    'fuel_gazebo_exploration', 'exploration_manager'):
        proc = subprocess.run(['rospack', 'find', package], stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True, timeout=5)
        source_paths[package] = proc.stdout.strip() if proc.returncode == 0 else None
    summary['package_paths'] = source_paths
    try:
        from gazebo_msgs.srv import GetWorldProperties, GetModelState
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=2.0)
        world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
        summary['gazebo'] = {'success': world.success, 'model_names': world.model_names,
                             'sim_time': world.sim_time, 'poses': {}}
        for model_name in (mav, 'depth_test_box', 'fuel_official_office'):
            if model_name not in world.model_names:
                continue
            state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(model_name, 'world')
            if state.success:
                summary['gazebo']['poses'][model_name] = genpy.message.strify_message(state.pose)
    except Exception as exc:
        summary['gazebo_error'] = str(exc)
    files = {'runtime/robot_description.urdf': urdf.encode()}
    for key, message in messages.items():
        files['runtime/' + key + '.yaml'] = message.encode()
    return summary, files


def main():
    parser = argparse.ArgumentParser(description='只读收集当前悬停配置，不发送任何飞行指令。')
    parser.add_argument('--mav-name', default='hummingbird')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--timeout', type=float, default=8.0)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 30:
        parser.error('--timeout must be between 1 and 30 wall-clock seconds')
    if not args.mav_name.replace('_', '').isalnum():
        parser.error('Invalid robot name')
    socket.setdefaulttimeout(5.0)
    files, warnings = launch_files(Path.home())
    try:
        summary, runtime_files = capture_ros(args.mav_name, args.timeout)
    except Exception as exc:
        parser.exit(1, '无法收集：' + str(exc) + '\n请在已 source ROS、Gazebo 正在悬停的终端执行。\n')
    files.update(runtime_files)
    summary['warnings'] = warnings
    summary['captured_at_local'] = datetime.datetime.now().astimezone().isoformat()
    summary['read_only'] = True
    summary['files_sha256'] = {key: hashlib.sha256(data).hexdigest() for key, data in files.items()}
    files['report.json'] = json.dumps(summary, indent=2, ensure_ascii=False).encode()
    name = 'fuel_official_inputs_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.zip'
    target = (args.output or (Path.home() / name)).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents overwriting an earlier diagnostic.
    with target.open('xb') as stream:
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as archive:
            for key in sorted(files):
                archive.writestr(key, files[key])
    print('只读资料已保存：' + str(target))
    if summary['missing_messages']:
        print('缺少实时消息：' + ', '.join(summary['missing_messages']))
    print('请提供这个 ZIP，用来核对真实相机位置和已有悬停启动配置。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
