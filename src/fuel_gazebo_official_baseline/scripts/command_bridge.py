#!/usr/bin/python3
"""Adapt official FUEL commands to RotorS without gating on replanning states.

The planner owns trajectory selection and switching. This process converts
messages and frames, runs the original vertical initialization explicitly, and
holds the last measured pose after an explicit stop or loss of valid inputs.
It never changes the FUEL state machine or replaces a valid command merely
because the planner is in PLAN_TRAJ or PUB_TRAJ.
"""
import copy
from collections import deque
import json
import math
import threading
import time
from pathlib import Path as FilePath
import sys

import numpy as np
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped, Transform, Twist
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

sys.path.insert(0, str(FilePath(__file__).resolve().parent))
from command_math import rotation_xyzw, startup_height, validate_command, world_twist


def xyz(v):
    return [v.x, v.y, v.z]


def xyzw(q):
    return [q.x, q.y, q.z, q.w]


def assign(v, values):
    v.x, v.y, v.z = map(float, values)


def pose_yaw(odom):
    rot = rotation_xyzw(xyzw(odom.pose.pose.orientation))
    return math.atan2(rot[1, 0], rot[0, 0])


class CommandBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.prepare_done = threading.Event()
        self.start_ack = threading.Event()
        self.ns = rospy.get_namespace().rstrip('/')
        self.master = rosgraph.Master(rospy.get_name())
        self.mode = 'WAIT_DATA'
        self.reason = 'Waiting for measured data; no exploration start has been sent.'
        self.odom = None
        self.odom_stamp = self.odom_wall = None
        self.sensor_ok = False
        self.sensor_wall = 0.0
        self.clock_stamp = None
        self.clock_wall = 0.0
        self.clock_generation = 0
        self.graph_ok = self.output_ok = False
        self.graph_reason = 'Waiting for ROS graph'
        self.state = 'INIT'
        self.state_wall = 0.0
        self.start_region_ok = False
        self.start_region_wall = 0.0
        self.hold = None
        self.command = None
        self.command_wall = 0.0
        self.command_stamp = 0.0
        self.last_trajectory_id = -1
        self.pending_commands = deque()
        self.timing = {'received': 0, 'accepted': 0}
        self.first_stop = None
        self.prepare_begin = self.prepare_wall = self.peak_z = None
        self.settle_begin = None
        self.start_wall = self.start_stamp = None
        self.started = False
        self.init_done = False
        self.path = Path()
        self.path.header.frame_id = 'world'
        self.path_stamp = -1.0
        self.command_topic = rospy.get_param('~command_topic', '/hummingbird/command/trajectory')
        self.odom_topic = rospy.get_param('~odom_topic', '/hummingbird/odometry_sensor1/odometry')
        self.controller_node = rospy.get_param('~controller_node', '/hummingbird/lee_position_controller_node')
        self.sensor_node = self.ns + '/sensor_bridge'
        self.traj_node = self.ns + '/traj_server'
        self.init = [float(rospy.get_param('~init_'+axis, val))
                     for axis, val in zip('xyz', [0., 0., 1.])]
        self.bounds = [rospy.get_param('~command_min', [-10., -15., 0.]),
                       rospy.get_param('~command_max', [10., 15., 2.5])]
        self.max_speed = float(rospy.get_param('~max_command_speed', 3.0))
        self.max_acceleration = float(rospy.get_param('~max_command_acceleration', 6.0))
        self.max_yaw_rate = float(rospy.get_param('~max_command_yaw_rate', 3.0))
        self.output = rospy.Publisher(self.command_topic, MultiDOFJointTrajectory, queue_size=1)
        self.odom_output = rospy.Publisher(self.ns+'/odom_world', Odometry, queue_size=5)
        self.trigger = rospy.Publisher(self.ns+'/trigger', Path, queue_size=1, latch=False)
        self.ready_output = rospy.Publisher(self.ns+'/command_ready', Bool, queue_size=1, latch=True)
        self.init_output = rospy.Publisher(self.ns+'/init_done', Bool, queue_size=1, latch=True)
        self.status_output = rospy.Publisher(self.ns+'/command_status', String, queue_size=1, latch=True)
        self.timing_output = rospy.Publisher(self.ns+'/command_timing', String, queue_size=1, latch=True)
        self.path_output = rospy.Publisher(self.ns+'/actual_path', Path, queue_size=1)
        self.ready_output.publish(Bool(data=False))
        self.init_output.publish(Bool(data=False))
        self.subs = [
            rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=5),
            rospy.Subscriber(self.ns+'/sensor_ready', Bool, self.on_sensor, queue_size=1),
            rospy.Subscriber(self.ns+'/planner/state', String, self.on_state, queue_size=10),
            rospy.Subscriber(self.ns+'/planner/start_region_ready', Bool, self.on_start_region, queue_size=1),
            rospy.Subscriber(self.ns+'/position_cmd', PositionCommand, self.on_command, queue_size=1, tcp_nodelay=True),
            rospy.Subscriber(self.ns+'/stop', Empty, self.on_stop, queue_size=1),
            rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=5)]
        self.services = [rospy.Service('~prepare', Trigger, self.prepare_service),
                         rospy.Service('~start', Trigger, self.start_service),
                         rospy.Service('~stop', Trigger, self.stop_service)]
        rospy.on_shutdown(self.on_shutdown)
        self.graph_worker = threading.Thread(target=self.graph_loop, daemon=True)
        self.worker = threading.Thread(target=self.control_loop, daemon=True)
        self.graph_worker.start()
        self.worker.start()
        rospy.loginfo('Official FUEL command adapter: preparation and exploration require explicit service calls.')

    def send(self, publisher, message):
        try:
            publisher.publish(message)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def capture_hold(self):
        if self.odom is not None:
            self.hold = (xyz(self.odom.pose.pose.position), pose_yaw(self.odom))

    def halt(self, reason, reset=False):
        # Called with the lock held. Faults latch until this launch is restarted.
        if self.first_stop is None:
            self.first_stop = {'reason': reason, 'mode': self.mode,
                               'clock': rospy.Time.now().to_sec(),
                               'accepted_stamp': self.command_stamp,
                               'accepted_wall_age': time.monotonic()-self.command_wall,
                               'trajectory_id': self.last_trajectory_id,
                               'last_packet': copy.deepcopy(self.timing.get('last_packet'))}
            rospy.logerr_throttle(2., 'Command adapter first stop: '+json.dumps(self.first_stop, sort_keys=True))
        reason = self.first_stop['reason']
        if reset:
            self.clock_generation += 1
            self.odom = None
            self.odom_stamp = self.odom_wall = None
            self.hold = None
            self.sensor_ok = self.start_region_ok = False
            self.path = Path()
            self.path.header.frame_id = 'world'
            self.path_stamp = -1.
        elif self.mode != 'HALTED':
            self.capture_hold()
        self.mode, self.reason = 'HALTED', reason
        self.command = None
        self.pending_commands.clear()
        self.started = self.init_done = False
        self.prepare_done.set()
        self.start_ack.set()
        self.send(self.init_output, Bool(data=False))
        rospy.logerr_throttle(2., reason)

    def on_clock(self, msg):
        with self.lock:
            stamp = msg.clock.to_sec()
            if self.clock_stamp is not None and stamp < self.clock_stamp-1e-6:
                self.halt('Gazebo clock restarted; restart this launch to clear the FUEL map.', reset=True)
            if self.clock_stamp is None or stamp != self.clock_stamp:
                self.clock_wall = time.monotonic()
            self.clock_stamp = stamp

    def on_odom(self, msg):
        with self.lock:
            try:
                stamp = msg.header.stamp.to_sec()
                now = rospy.Time.now().to_sec()
                if msg.header.frame_id.lstrip('/') != 'world':
                    raise ValueError('Measured odometry must be expressed in world')
                if not np.isfinite(xyz(msg.pose.pose.position)).all():
                    raise ValueError('Invalid measured position')
                # A delayed packet from before a clock reset must never become a hold target.
                if stamp <= 0 or stamp > now+.05:
                    return
                if self.odom_stamp is not None and stamp < self.odom_stamp-1e-6:
                    self.halt('Measured odometry time restarted; restart this launch.', reset=True)
                    return
                velocity, angular, covariance = world_twist(
                    xyzw(msg.pose.pose.orientation), xyz(msg.twist.twist.linear),
                    xyz(msg.twist.twist.angular), msg.twist.covariance)
                output = copy.deepcopy(msg)
                assign(output.twist.twist.linear, velocity)
                assign(output.twist.twist.angular, angular)
                output.twist.covariance = covariance.reshape(-1).tolist()
                # Republish only advancing measurements. Duplicate packets do not refresh health.
                if self.odom_stamp is None or stamp > self.odom_stamp+1e-9:
                    self.odom, self.odom_stamp, self.odom_wall = output, stamp, time.monotonic()
                    self.send(self.odom_output, output)
                    if self.mode == 'HALTED' and self.hold is None:
                        self.capture_hold()
                    if stamp-self.path_stamp >= .1:
                        pose = PoseStamped()
                        pose.header, pose.pose = copy.copy(msg.header), copy.deepcopy(msg.pose.pose)
                        self.path.poses.append(pose)
                        self.path.poses = self.path.poses[-12000:]
                        self.path.header.stamp = msg.header.stamp
                        self.send(self.path_output, self.path)
                        self.path_stamp = stamp
            except (ValueError, TypeError) as exc:
                self.halt(str(exc))

    def on_sensor(self, msg):
        with self.lock:
            self.sensor_ok = bool(msg.data)
            self.sensor_wall = time.monotonic()

    def on_start_region(self, msg):
        with self.lock:
            self.start_region_ok = bool(msg.data)
            self.start_region_wall = time.monotonic()

    def on_state(self, msg):
        with self.lock:
            self.state, self.state_wall = msg.data, time.monotonic()
            if self.mode == 'START_PENDING' and self.state in {'PLAN_TRAJ', 'PUB_TRAJ', 'EXEC_TRAJ', 'FINISH'}:
                self.mode, self.started = 'RUNNING', True
                self.reason = 'Official planner acknowledged the single exploration trigger.'
                self.start_ack.set()
            # Deliberately no PLAN_TRAJ/PUB_TRAJ/EXEC_TRAJ command gate here.

    def on_command(self, msg):
        with self.lock:
            if self.mode not in {'START_PENDING', 'RUNNING'}:
                return
            try:
                stamp = msg.header.stamp.to_sec()
                now, wall = rospy.Time.now().to_sec(), time.monotonic()
                age = now-stamp
                self.timing['received'] += 1
                self.timing['last_packet'] = {'stamp': stamp, 'clock': now, 'age': age,
                                              'trajectory_id': msg.trajectory_id,
                                              'yaw': msg.yaw, 'yaw_rate': msg.yaw_dot,
                                              'speed': float(np.linalg.norm(xyz(msg.velocity))),
                                              'acceleration': float(np.linalg.norm(xyz(msg.acceleration)))}
                if not math.isfinite(stamp) or stamp <= 0:
                    self.count_timing('dropped_invalid_stamp')
                    return
                # One late packet is not a loss of the current command stream.
                # Rejected packets never refresh accepted stamp/receipt/id.
                if age > .3:
                    self.count_timing('dropped_stale')
                    return
                if age < -.3:
                    self.count_timing('dropped_far_future')
                    return
                if msg.header.frame_id.lstrip('/') != 'world':
                    raise ValueError('PositionCommand must use world coordinates')
                values = (xyz(msg.position), xyz(msg.velocity), xyz(msg.acceleration), msg.yaw, msg.yaw_dot)
                validate_command(*values, self.bounds, self.max_speed, self.max_acceleration, self.max_yaw_rate)
                if stamp <= self.command_stamp+1e-9:
                    self.count_timing('dropped_duplicate' if abs(stamp-self.command_stamp)<1e-9 else 'dropped_out_of_order')
                    return
                if age < 0:
                    # Keep earlier future samples too: replacing only the newest
                    # sample can starve forever when another process's clock leads.
                    if any(abs(item[0]-stamp)<1e-9 for item in self.pending_commands):
                        self.count_timing('dropped_duplicate')
                        return
                    if len(self.pending_commands) >= 128:
                        self.pending_commands.popleft()
                        self.count_timing('dropped_overflow')
                    self.pending_commands.append((stamp, msg.trajectory_id, wall, values))
                    self.count_timing('buffered_future')
                    return
                self.accept_command(stamp, msg.trajectory_id, wall, values)
            except (ValueError, TypeError) as exc:
                self.halt(str(exc))

    def count_timing(self, key):
        self.timing[key] = self.timing.get(key, 0)+1

    def accept_command(self, stamp, trajectory_id, received_wall, values):
        if stamp <= self.command_stamp+1e-9:
            self.count_timing('dropped_duplicate' if abs(stamp-self.command_stamp)<1e-9 else 'dropped_out_of_order')
            return
        if trajectory_id < self.last_trajectory_id:
            raise ValueError('PositionCommand trajectory id went backwards')
        self.command, self.command_stamp = values, stamp
        self.command_wall = received_wall
        self.last_trajectory_id = trajectory_id
        self.count_timing('accepted')

    def promote_commands(self, wall, now):
        eligible, pending = [], deque()
        for item in self.pending_commands:
            stamp, trajectory_id, received_wall, values = item
            if now-stamp > .3 or wall-received_wall > 1.:
                self.count_timing('dropped_expired_buffer')
            elif stamp <= now:
                eligible.append(item)
            else:
                pending.append(item)
        self.pending_commands = pending
        # Do not stamp a buffered command with its promotion time.
        if eligible:
            stamp, trajectory_id, received_wall, values = max(eligible, key=lambda item:item[0])
            self.accept_command(stamp, trajectory_id, received_wall, values)

    def data_problem(self, wall, now):
        if not self.graph_ok:
            return self.graph_reason
        if self.clock_stamp is None or wall-self.clock_wall > 1.:
            return 'Gazebo clock is not advancing'
        if self.odom is None or self.odom_wall is None or wall-self.odom_wall > 1.:
            return 'Measured odometry stopped'
        if not -.05 <= now-self.odom_stamp <= .3:
            return 'Measured odometry timestamp is not current'
        if not self.sensor_ok or wall-self.sensor_wall > 1.5:
            return 'Paired depth/pose input is not ready or stopped'
        return None

    def ready_problem(self):
        wall, now = time.monotonic(), rospy.Time.now().to_sec()
        if self.mode == 'HALTED':
            return self.reason
        problem = self.data_problem(wall, now)
        if problem:
            return problem
        if self.state != 'WAIT_TRIGGER' or wall-self.state_wall > 2.:
            return 'Official planner is not freshly reporting WAIT_TRIGGER'
        if self.trigger.get_num_connections() != 1:
            return 'Exploration trigger must have exactly one planner subscriber'
        return None

    def prepare_service(self, _request):
        with self.lock:
            if self.init_done:
                return TriggerResponse(True, 'Official vertical initialization is already complete.')
            if self.mode != 'WAIT_DATA':
                return TriggerResponse(False, 'Cannot prepare in '+self.mode+': '+self.reason)
            problem = self.ready_problem()
            if problem:
                return TriggerResponse(False, problem)
            if np.linalg.norm(np.asarray(xyz(self.odom.pose.pose.position))-self.init) > .2:
                return TriggerResponse(False, 'Measured vehicle must be at the configured (0, 0, 1) startup pose.')
            if abs(pose_yaw(self.odom)) > .15:
                return TriggerResponse(False, 'Startup yaw must be close to zero, as in the official demo.')
            self.mode, self.reason = 'PREPARING', 'Running the official upward/downward initialization.'
            self.prepare_begin = rospy.Time.now().to_sec()
            self.prepare_wall = time.monotonic()
            self.peak_z = self.odom.pose.pose.position.z
            self.settle_begin = None
            self.prepare_done.clear()
            self.command = None
            self.pending_commands.clear()
        # Wait on wall time: paused simulation must not hang a service forever.
        if not self.prepare_done.wait(35.):
            with self.lock:
                self.halt('Official initialization timed out; measured hover was not recovered.')
        with self.lock:
            return TriggerResponse(self.init_done, self.reason)

    def start_service(self, _request):
        with self.lock:
            if self.started or self.mode == 'START_PENDING':
                return TriggerResponse(False, 'Exploration has already been triggered; no second trigger was sent.')
            problem = self.ready_problem()
            if problem:
                return TriggerResponse(False, problem)
            if not self.init_done or self.mode != 'INITIALIZED':
                return TriggerResponse(False, 'Run the prepare service and wait for measured hover first.')
            if not self.start_region_ok or time.monotonic()-self.start_region_wall > 1.5:
                return TriggerResponse(False, 'No freshly observed first planning edge. Read planner/start_region_report.')
            if np.linalg.norm(np.asarray(xyz(self.odom.pose.pose.position))-self.init) > .15:
                return TriggerResponse(False, 'Vehicle has left the initialized hover pose.')
            if np.linalg.norm(xyz(self.odom.twist.twist.linear)) > .15:
                return TriggerResponse(False, 'Vehicle must be hovering before the one exploration trigger.')
            self.mode, self.reason = 'START_PENDING', 'One exploration trigger sent; waiting for planner acknowledgement.'
            self.start_wall, self.start_stamp = time.monotonic(), rospy.Time.now().to_sec()
            self.command = None
            self.command_stamp, self.command_wall = 0., 0.
            self.last_trajectory_id = -1
            self.pending_commands.clear()
            self.start_ack.clear()
            path, pose = Path(), PoseStamped()
            path.header.stamp, path.header.frame_id = rospy.Time.now(), 'world'
            pose.header = copy.copy(path.header)
            pose.pose = copy.deepcopy(self.odom.pose.pose)
            path.poses = [pose]
            self.send(self.trigger, path)
        if not self.start_ack.wait(3.):
            with self.lock:
                self.halt('Planner did not acknowledge the single trigger; restart the launch before retrying.')
        with self.lock:
            return TriggerResponse(self.started and self.mode == 'RUNNING', self.reason)

    def on_stop(self, _msg):
        with self.lock:
            self.halt('Explicit stop received. Holding the measured pose; restart this launch before another flight.')

    def stop_service(self, _request):
        self.on_stop(None)
        return TriggerResponse(True, self.reason)

    def graph_loop(self):
        while not self.quit.is_set() and not rospy.is_shutdown():
            try:
                pubs, subs, _ = self.master.getSystemState()
                pubs, subs = dict(pubs), dict(subs)
                checks = [
                    (set(pubs.get(self.command_topic, [])) == {rospy.get_name()}, 'RotorS command topic must have this adapter as its sole publisher'),
                    (self.controller_node in subs.get(self.command_topic, []), 'RotorS controller is not subscribing to its command topic'),
                    (set(pubs.get(self.ns+'/position_cmd', [])) == {self.traj_node}, 'PositionCommand must have exactly the official trajectory server publisher'),
                    (set(pubs.get(self.ns+'/sensor_ready', [])) == {self.sensor_node}, 'Sensor readiness must have exactly the sensor bridge publisher'),
                    (set(pubs.get(self.ns+'/odom_world', [])) == {rospy.get_name()}, 'World-frame odometry must have exactly this adapter publisher'),
                    (len(pubs.get(self.odom_topic, [])) == 1, 'Measured odometry must have exactly one publisher'),
                    (set(pubs.get(self.ns+'/trigger', [])) == {rospy.get_name()}, 'Exploration trigger must have exactly this adapter publisher')]
                reason = next((reason for valid, reason in checks if not valid), '')
                with self.lock:
                    self.graph_ok, self.graph_reason = not reason, reason
                    self.output_ok = checks[0][0] and checks[1][0]
            except Exception as exc:
                with self.lock:
                    self.graph_ok, self.graph_reason = False, 'Cannot check ROS connections: '+str(exc)
            self.quit.wait(.5)

    def publish_target(self, p, v, a, yaw, yaw_dot):
        msg = MultiDOFJointTrajectory()
        msg.header.stamp, msg.header.frame_id = rospy.Time.now(), 'world'
        point, transform, vel, acc = MultiDOFJointTrajectoryPoint(), Transform(), Twist(), Twist()
        assign(transform.translation, p)
        transform.rotation.z, transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        assign(vel.linear, v)
        vel.angular.z = float(yaw_dot)
        assign(acc.linear, a)
        point.transforms, point.velocities, point.accelerations = [transform], [vel], [acc]
        point.time_from_start = rospy.Duration(0)
        msg.points = [point]
        self.send(self.output, msg)

    def tick(self, wall, now):
        """One wall-timed cycle, separated from the thread for deterministic tests."""
        with self.lock:
            if self.mode in {'START_PENDING', 'RUNNING'}:
                try:
                    self.promote_commands(wall, now)
                except (ValueError, TypeError) as exc:
                    self.halt(str(exc))
            problem = self.data_problem(wall, now)
            if self.mode not in {'WAIT_DATA', 'HALTED'} and problem:
                self.halt(problem)
            if self.mode == 'WAIT_DATA' and not problem and self.hold is None:
                self.capture_hold()
            if self.mode == 'PREPARING':
                elapsed = now-self.prepare_begin
                if elapsed < -1e-6:
                    self.halt('Clock restarted during official initialization.', reset=True)
                elif wall-self.prepare_wall > 30. or elapsed > 20.:
                    self.halt('Official initialization did not return to measured hover in time.')
                else:
                    self.peak_z = max(self.peak_z, self.odom.pose.pose.position.z)
                    if elapsed < 2.:
                        self.publish_target([self.init[0], self.init[1], startup_height(elapsed, self.init[2])],
                                            [0., 0., 0.], [0., 0., 0.], 0., 0.)
                        return
                    self.hold = (list(self.init), 0.)
                    error = np.linalg.norm(np.asarray(xyz(self.odom.pose.pose.position))-self.init)
                    speed = np.linalg.norm(xyz(self.odom.twist.twist.linear))
                    stable = error < .12 and speed < .12 and abs(pose_yaw(self.odom)) < .12
                    if stable:
                        if self.settle_begin is None:
                            self.settle_begin = now
                        if now-self.settle_begin >= 1.:
                            if self.peak_z < self.init[2]+.5:
                                self.halt('The vertical initialization was commanded but measured ascent was insufficient.')
                            else:
                                self.mode, self.init_done = 'INITIALIZED', True
                                self.reason = 'Official vertical initialization complete; measured hover recovered. Exploration has not started.'
                                self.prepare_done.set()
                                self.send(self.init_output, Bool(data=True))
                    else:
                        self.settle_begin = None
            if self.mode == 'RUNNING':
                if self.command is not None:
                    if wall-self.command_wall > 1. or not -.05 <= now-self.command_stamp <= .3:
                        self.halt('Official trajectory commands stopped or became stale: command_age=%.6f s, receipt_age=%.6f s.' %
                                  (now-self.command_stamp, wall-self.command_wall))
                    else:
                        # Existing commands continue during PLAN_TRAJ and PUB_TRAJ.
                        self.publish_target(*self.command)
                        return
                elif self.state == 'FINISH':
                    self.mode, self.reason = 'FINISHED', 'Official planner finished without publishing a trajectory.'
                    self.capture_hold()
                elif wall-self.start_wall > 15.:
                    self.halt('No trajectory command arrived within 15 wall-clock seconds of the accepted trigger.')
            if self.hold is not None and self.output_ok:
                self.publish_target(self.hold[0], [0., 0., 0.], [0., 0., 0.], self.hold[1], 0.)

    def control_loop(self):
        last_status = 0.
        while not self.quit.is_set() and not rospy.is_shutdown():
            try:
                with self.lock:
                    # Sample the clock after acquiring the command lock, so a
                    # newer callback cannot make this tick's snapshot obsolete.
                    wall, now = time.monotonic(), rospy.Time.now().to_sec()
                    self.tick(wall, now)
                    if wall-last_status >= .2:
                        ready = self.ready_problem() is None
                        self.send(self.ready_output, Bool(data=ready))
                        self.send(self.init_output, Bool(data=self.init_done))
                        self.send(self.status_output, String(data=self.mode+': '+self.reason))
                        timing = dict(self.timing, pending=len(self.pending_commands), first_stop=self.first_stop,
                                      accepted_stamp=self.command_stamp, accepted_wall_age=wall-self.command_wall)
                        self.send(self.timing_output, String(data=json.dumps(timing, sort_keys=True)))
                        last_status = wall
            except Exception as exc:
                with self.lock:
                    self.halt('Command adapter error: '+str(exc))
            self.quit.wait(.01)

    def on_shutdown(self):
        self.quit.set()
        with self.lock:
            self.capture_hold()
            if self.hold is not None and self.output_ok:
                try:
                    self.publish_target(self.hold[0], [0., 0., 0.], [0., 0., 0.], self.hold[1], 0.)
                except rospy.ROSException:
                    pass


if __name__ == '__main__':
    rospy.init_node('command_bridge')
    bridge = CommandBridge()
    rospy.spin()
