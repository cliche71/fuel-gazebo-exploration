#!/usr/bin/env python3
"""Start the office simulation and planner; exploration is triggered separately."""
from pathlib import Path
import argparse, os, signal, subprocess, time, datetime
import rospy
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description='Launch FUEL in the office scene and wait for user prepare/start.')
parser.add_argument('--gui',choices=['true','false'],default='true')
parser.add_argument('--seed',type=int,default=20261029)
parser.add_argument('--profile',default='rotors_1mps')
args=parser.parse_args()
session=ROOT/'logs'/datetime.datetime.now().strftime('run_%Y%m%d_%H%M%S_%f')
trace=Path(os.environ.get('FUEL_REGRESSION_TRACE_DIR') or str(session/'internal')).expanduser()
trace.mkdir(parents=True,exist_ok=True)
os.environ['FUEL_REGRESSION_TRACE_DIR']=str(trace.resolve())
os.environ.setdefault('ROS_LOG_DIR',str(session/'ros'))
print('Runtime logs: '+os.environ['FUEL_REGRESSION_TRACE_DIR'],flush=True)
robot=Path.home()/'.ros/fuel_gazebo/robot.urdf'
subprocess.check_call(['/usr/bin/python3',str(ROOT/'src/fuel_gazebo_official_baseline/scripts/make_official_robot.py'),'--output',str(robot)])
children=[]
def launch(command):
 child=subprocess.Popen(command,start_new_session=True);children.append(child);return child
def cleanup():
 for child in reversed(children):
  if child.poll() is None:
   try:os.killpg(child.pid,signal.SIGINT)
   except ProcessLookupError:continue
   try:child.wait(20)
   except subprocess.TimeoutExpired:
    try:os.killpg(child.pid,signal.SIGTERM);child.wait(5)
    except (ProcessLookupError,subprocess.TimeoutExpired):
     if child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait()
try:
 launch(['roslaunch','fuel_gazebo_official_baseline','simulation.launch','gui:='+args.gui,'seed:='+str(args.seed),'robot_file:='+str(robot)])
 rospy.init_node('fuel_current_bringup',disable_signals=True)
 measured={'hover':False}
 def odom(msg):measured['hover']=abs(msg.pose.pose.position.z-1.)<.12
 sub=rospy.Subscriber('/hummingbird/odometry_sensor1/odometry',Odometry,odom,queue_size=1)
 deadline=time.monotonic()+60
 while not measured['hover']:
  if time.monotonic()>deadline or children[0].poll() is not None:raise RuntimeError('Original independent hover did not become available; inspect simulation output.')
  time.sleep(.05)
 sub.unregister()
 launch(['roslaunch','fuel_gazebo_official_baseline','planner.launch','rviz:='+args.gui,'profile:='+args.profile])
 print('Hover ready. Call prepare, then start once as documented.',flush=True)
 while all(p.poll() is None for p in children):time.sleep(.2)
 raise RuntimeError('A launch exited; restart the independent simulation before another exploration.')
except KeyboardInterrupt:pass
finally:cleanup()
