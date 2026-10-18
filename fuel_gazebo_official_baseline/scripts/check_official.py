#!/usr/bin/env python3
"""Read-only runtime report for the official Gazebo baseline."""
import argparse
import collections
import datetime
import json
import math
from pathlib import Path
import threading
import time
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))


def main():
    parser = argparse.ArgumentParser(description='只读检查官方对照版本；不会准备、起飞或发送探索指令。')
    parser.add_argument('--phase', choices=('camera', 'ready', 'flight'), default='camera')
    parser.add_argument('--seconds', type=float, default=8.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 120:
        parser.error('--seconds must be between 5 and 120 wall seconds')
    import rospy
    import rosgraph
    import rosnode
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from quadrotor_msgs.msg import PositionCommand
    from sensor_math import camera_transform

    rospy.init_node('check_official_baseline', anonymous=True, argv=['check_official_baseline'])
    lock = threading.Lock()
    counts = collections.Counter()
    latest, stamps, positions, ids = {}, collections.defaultdict(list), [], set()
    subscribers = []

    def callback(msg, key):
        with lock:
            counts[key] += 1
            if hasattr(msg, 'header'):
                stamps[key].append(msg.header.stamp.to_sec())
            if key == 'odom':
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                positions.append([p.x,p.y,p.z])
                latest[key] = {'position':positions[-1], 'speed':math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z), 'frame':msg.header.frame_id}
            elif key == 'info':
                latest[key] = {'K':list(msg.K),'width':msg.width,'height':msg.height,'frame':msg.header.frame_id}
            elif key == 'depth':
                if msg.encoding!='32FC1' or msg.step%4 or len(msg.data)!=msg.height*msg.step:
                    latest[key]={'encoding':msg.encoding,'error':'Unexpected depth layout'}
                    return
                arr = np.frombuffer(msg.data,dtype=('>f4' if msg.is_bigendian else '<f4'))
                rows = arr.reshape(msg.height,msg.step//4)[:,:msg.width]
                center=rows[msg.height//2-5:msg.height//2+5,msg.width//2-5:msg.width//2+5]
                valid=center[np.isfinite(center)&(center>0)]
                latest[key]={'encoding':msg.encoding,'width':msg.width,'height':msg.height,'frame':msg.header.frame_id,
                             'center_depth_median':float(np.median(valid)) if len(valid) else None}
            elif key == 'clock':
                stamps[key].append(msg.clock.to_sec())
                latest[key]=msg.clock.to_sec()
            elif key == 'position_cmd':
                ids.add(msg.trajectory_id)
            else:
                latest[key]=msg.data

    specifications={
        'odom':('/hummingbird/odometry_sensor1/odometry',Odometry),
        'info':('/hummingbird/vi_sensor/camera_depth/depth/camera_info',CameraInfo),
        'depth':('/hummingbird/vi_sensor/camera_depth/depth/disparity',Image),
        'clock':('/clock',Clock),
        'sensor_ready':('/fuel_official/sensor_ready',Bool),
        'sensor_report':('/fuel_official/sensor_report',String),
        'state':('/fuel_official/planner/state',String),
        'region':('/fuel_official/planner/start_region_report',String),
        'init_done':('/fuel_official/init_done',Bool),
        'command_ready':('/fuel_official/command_ready',Bool),
        'command_status':('/fuel_official/command_status',String),
        'command_timing':('/fuel_official/command_timing',String),
        'position_cmd':('/fuel_official/position_cmd',PositionCommand),
    }
    for key,(topic,kind) in specifications.items():
        subscribers.append(rospy.Subscriber(topic,kind,callback,callback_args=key,queue_size=1,buff_size=4000000 if key=='depth' else 65536))
    print('Checking '+args.phase+' for '+str(args.seconds)+' wall-clock seconds...')
    deadline=time.monotonic()+args.seconds
    while time.monotonic()<deadline and not rospy.is_shutdown():
        time.sleep(.05)
    for sub in subscribers: sub.unregister()
    errors=[]
    with lock:
        report={'phase':args.phase,'captured_at_local':datetime.datetime.now().astimezone().isoformat(),
                'counts':dict(counts),'latest':latest,'trajectory_ids':sorted(ids),
                'rates_hz':{key:counts[key]/args.seconds for key in ('odom','info','depth','clock')}}
    for key,minimum in (('odom',5),('depth',1),('info',1),('clock',5)):
        if counts[key]/args.seconds<minimum or len(set(stamps[key]))<3:
            errors.append(key+' is missing, too slow, or has stopped timestamps')
    info=latest.get('info',{})
    if info.get('frame','').lstrip('/')!='hummingbird/fuel_official_camera_optical_link': errors.append('The old VI optical frame is still being used')
    if info.get('width')!=640 or info.get('height')!=480 or not np.isclose(info.get('K',[0]*9)[0],387.229248046875,atol=.01): errors.append('New camera calibration is not active')
    depth=latest.get('depth',{})
    if depth.get('encoding')!='32FC1' or depth.get('error'): errors.append('Depth payload has an unexpected format')
    if args.phase=='camera' and depth.get('center_depth_median') is None: errors.append('Camera centre has no finite wall observation; check the Gazebo camera view')
    mass=rospy.get_param('/hummingbird/lee_position_controller_node/mass',None)
    report['controller_mass']=mass
    if mass is None or abs(mass-.87607)>1e-5: errors.append('Controller mass differs from the working VI model')
    try:
        tf=camera_transform(rospy.get_param('/hummingbird/robot_description'),
                            'hummingbird/odometry_sensor1_link','hummingbird/fuel_official_camera_optical_link')
        report['camera_transform_from_measured_odometry']=tf.tolist()
        if np.linalg.norm(tf[:3,3])>1e-7: errors.append('Camera optical frame is offset from odometry')
    except Exception as exc: errors.append('Running robot description: '+str(exc))
    nodes=rosnode.get_node_names()
    report['nodes']=nodes
    if any(x.startswith('/fuel_exploration/') or x.startswith('/fuel_navigation/') for x in nodes): errors.append('Old navigation/exploration nodes are still running')
    pubs,subs,_=rosgraph.Master(rospy.get_name()).getSystemState()
    report['command_publishers']=dict(pubs).get('/hummingbird/command/trajectory',[])
    if args.phase=='camera':
        if latest.get('odom',{}).get('speed',99)>.15 or abs(latest.get('odom',{}).get('position',[0,0,99])[2]-1)>.2: errors.append('Vehicle is not hovering near one metre')
    if args.phase in ('ready','flight'):
        if latest.get('sensor_ready') is not True: errors.append('Measured sensor pairing is not ready')
        if latest.get('init_done') is not True: errors.append('Official vertical initialization has not completed')
        if set(report['command_publishers'])!={'/fuel_official/command_bridge'}: errors.append('Flight command publisher is not unique')
        if str(latest.get('command_status','')).startswith('HALTED'): errors.append('Command bridge halted: '+latest['command_status'])
    if args.phase=='ready':
        try:
            region=json.loads(latest.get('region','{}'))
            report['start_region']=region
            if region.get('clear_edges',0)<1 or region.get('frontier_groups',0)<1: errors.append('No observed first edge or no exploration frontier')
        except (ValueError,TypeError): errors.append('Invalid start-region report')
        if latest.get('state')!='WAIT_TRIGGER' or latest.get('command_ready') is not True: errors.append('Original planner/command adapter is not waiting ready')
    if args.phase=='flight':
        displacement=max((float(np.linalg.norm(np.array(p)-positions[0])) for p in positions),default=0.)
        report['measured_displacement_m']=displacement
        if displacement<.5: errors.append('Measured displacement during this recording is below 0.5 m')
        if len(ids)<2: errors.append('Fewer than two original trajectory IDs were observed')
    report['errors']=errors
    try:
        report['command_timing']=json.loads(latest.get('command_timing','{}'))
    except (ValueError,TypeError):
        report['command_timing']={'invalid_report':latest.get('command_timing')}
    report['result']='PASSED' if not errors else 'FAILED'
    path=Path.home()/('fuel_official_'+args.phase+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.txt')
    path.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print('OFFICIAL '+args.phase.upper()+' CHECK '+report['result'])
    for error in errors: print('- '+error)
    print('Report: '+str(path))
    return int(bool(errors))


if __name__=='__main__':
    raise SystemExit(main())
