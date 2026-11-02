#!/usr/bin/env python3
"""Relay measured Gazebo depth with its actual, interpolated camera pose."""
from collections import deque
import copy
import json
import math
import threading
import time
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from sensor_math import camera_transform, interpolate, quat_matrix, quat_product, quaternion, rotation_rpy


class SensorBridge:
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo, Image
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, String
        self.ros, self.PoseStamped, self.Bool, self.String = rospy, PoseStamped, Bool, String
        self.lock = threading.RLock()
        self.samples, self.pending = deque(maxlen=500), None
        self.pairs, self.last_pair_wall, self.last_info_wall = 0, 0., 0.
        self.last_depth_stamp, self.last_odom_stamp = None, None
        self.info, self.failed = None, ''
        self.stop_event = threading.Event()
        self.ready_pub = rospy.Publisher('/fuel_official/sensor_ready', Bool, queue_size=1, latch=True)
        self.report_pub = rospy.Publisher('/fuel_official/sensor_report', String, queue_size=1, latch=True)
        self.depth_pub = rospy.Publisher('/fuel_official/depth', Image, queue_size=2)
        self.pose_pub = rospy.Publisher('/fuel_official/sensor_pose', PoseStamped, queue_size=2)
        self.mav = rospy.get_param('~mav_name', 'hummingbird')
        self.camera_link = self.mav + '/fuel_official_camera_optical_link'
        xml = rospy.get_param('/'+self.mav+'/robot_description')
        transform = camera_transform(xml, self.mav+'/odometry_sensor1_link', self.camera_link)
        expected = rotation_rpy([-math.pi/2, 0, -math.pi/2])
        if np.linalg.norm(transform[:3, 3]) > 1e-7 or not np.allclose(transform[:3, :3], expected, atol=1e-7):
            raise ValueError('Official camera is not at the measured odometry origin with the expected optical axes.')
        self.offset = transform[:3, 3]
        # Quaternion for Rz(-pi/2) Rx(-pi/2), xyzw.
        self.optical_q = np.array([-.5, .5, -.5, .5])
        self.ready_pub.publish(Bool(data=False))
        rospy.Subscriber(rospy.get_param('~camera_info_topic', '/'+self.mav+'/vi_sensor/camera_depth/depth/camera_info'), CameraInfo, self.camera_info, queue_size=1)
        rospy.Subscriber(rospy.get_param('~depth_topic', '/'+self.mav+'/vi_sensor/camera_depth/depth/disparity'), Image, self.depth, queue_size=1, buff_size=4000000)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/'+self.mav+'/odometry_sensor1/odometry'), Odometry, self.odometry, queue_size=50)
        rospy.on_shutdown(self.shutdown)
        self.worker = threading.Thread(target=self.watchdog, daemon=True)
        self.worker.start()

    def fail(self, reason):
        self.failed = str(reason)
        self.ready_pub.publish(self.Bool(data=False))
        self.ros.logerr_throttle(2., 'Official sensor bridge: ' + self.failed)

    def camera_info(self, msg):
        with self.lock:
            if self.failed:
                return
            try:
                if msg.header.frame_id.lstrip('/') != self.camera_link or (msg.width, msg.height) != (640, 480):
                    raise ValueError('CameraInfo belongs to the old camera or has unexpected dimensions. Restart with simulation.launch.')
                values = np.asarray(msg.K, dtype=float)
                if not np.isfinite(values).all() or min(values[0], values[4]) < 100:
                    raise ValueError('Invalid camera calibration')
                if not np.allclose(msg.D, 0, atol=1e-9):
                    raise ValueError('This bridge expects the undistorted Gazebo camera')
                expected = [320.0/math.tan(math.radians(40.0)), 0, 320, 0, 240.0/math.tan(math.radians(30.0)), 240, 0, 0, 1]
                if not np.allclose(values, expected, rtol=1e-6, atol=1e-4):
                    raise ValueError("Actual camera K does not match paper-condition projection")
                if not np.allclose(msg.P, [values[0], 0, values[2], 0, 0, values[4], values[5], 0, 0, 0, 1, 0], rtol=1e-6, atol=1e-4):
                    raise ValueError("Actual camera P disagrees with measured K")
                current = (msg.width, msg.height, tuple(msg.K))
                if self.info and current != self.info:
                    raise ValueError('Camera calibration changed during this run; restart the planner')
                if not self.info:
                    self.info = current
                    fx, fy, cx, cy = values[0], values[4], values[2], values[5]
                    for key, value in dict(fx=fx, fy=fy, cx=cx, cy=cy).items():
                        self.ros.set_param('/fuel_official/planner/map_ros/'+key, float(value))
                    angles = {'top_angle': min(math.atan((cy-2)/fy), math.atan((msg.height-3-cy)/fy)),
                              'left_angle': math.atan((cx-2)/fx),
                              'right_angle': math.atan((msg.width-3-cx)/fx)}
                    for key, value in angles.items():
                        self.ros.set_param('/fuel_official/planner/perception_utils/'+key, float(value))
                        self.ros.set_param('/fuel_official/traj_server/perception_utils/'+key, float(value))
                    self.ros.set_param('/fuel_official/sensor_configured', True)
                self.last_info_wall = time.monotonic()
            except Exception as exc:
                self.fail(exc)

    def odometry(self, msg):
        with self.lock:
            if self.failed:
                return
            try:
                stamp = msg.header.stamp.to_sec()
                if self.last_odom_stamp is not None and stamp < self.last_odom_stamp-1e-6:
                    raise ValueError('Simulation time moved backwards; restart the official launch')
                if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
                    return
                if msg.header.frame_id.lstrip('/') != 'world':
                    raise ValueError('Measured odometry must be in world')
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                position = np.array([p.x, p.y, p.z])
                if not np.isfinite(position).all():
                    raise ValueError('Non-finite measured odometry')
                self.samples.append((stamp, position, quaternion([q.x, q.y, q.z, q.w])))
                self.last_odom_stamp = stamp
                self.pair()
            except Exception as exc:
                self.fail(exc)

    def depth(self, msg):
        with self.lock:
            if self.failed:
                return
            stamp = msg.header.stamp.to_sec()
            if self.last_depth_stamp is not None and stamp < self.last_depth_stamp-1e-6:
                self.fail('Depth clock moved backwards; restart official launch')
                return
            if self.last_depth_stamp is not None and stamp <= self.last_depth_stamp:
                return
            if msg.header.frame_id.lstrip('/') != self.camera_link or msg.encoding != '32FC1' or (msg.width, msg.height) != (640, 480) or msg.step < msg.width*4 or len(msg.data) != msg.step*msg.height:
                self.fail('Unexpected depth image/frame; the old VI camera must not feed the official planner')
                return
            self.last_depth_stamp, self.pending = stamp, msg
            self.pair()

    def pair(self):
        if self.pending is None or self.info is None:
            return
        image = self.pending
        measured = interpolate(self.samples, image.header.stamp.to_sec())
        if measured is None:
            return
        position, q = measured
        camera_position = position + quat_matrix(q) @ self.offset
        camera_q = quat_product(q, self.optical_q)
        pose = self.PoseStamped()
        pose.header.stamp, pose.header.frame_id = image.header.stamp, 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, camera_position)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, camera_q)
        self.pose_pub.publish(pose)
        self.depth_pub.publish(image)
        self.pending = None
        self.pairs += 1
        self.last_pair_wall = time.monotonic()

    def watchdog(self):
        while not self.stop_event.wait(.1):
            with self.lock:
                now = time.monotonic()
                ready = bool(not self.failed and self.pairs >= 5 and now-self.last_pair_wall < .8 and now-self.last_info_wall < 2.)
                self.ready_pub.publish(self.Bool(data=ready))
                self.report_pub.publish(self.String(data=json.dumps({
                    'ready': ready, 'pairs': self.pairs, 'camera_link': self.camera_link,
                    'camera_translation_from_odometry': self.offset.tolist(),
                    'calibration': self.info, 'failure': self.failed,
                    'pair_age_wall': now-self.last_pair_wall if self.pairs else None})))

    def shutdown(self):
        self.stop_event.set()
        self.ready_pub.publish(self.Bool(data=False))


def main():
    import rospy
    rospy.init_node('official_sensor_bridge')
    rospy.set_param('/fuel_official/sensor_configured', False)
    try:
        SensorBridge()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal('Cannot start official camera input: %s', exc)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
