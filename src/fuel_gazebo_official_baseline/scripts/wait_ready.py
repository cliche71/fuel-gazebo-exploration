#!/usr/bin/env python3
"""roslaunch prefix: defer C++ startup until measured calibration is installed."""
import os
from pathlib import Path
import sys
import time


def main():
    if len(sys.argv) < 2:
        raise SystemExit('Usage: wait_ready.py NODE_EXECUTABLE [node arguments]')
    workspace = Path(__file__).resolve().parents[3]
    executable = Path(sys.argv[1]).resolve()
    if not (workspace/'VERSION').is_file() or workspace not in executable.parents:
        raise SystemExit('Refusing to launch FUEL from another workspace. Source this repository env.sh after building it.')
    import rospy
    from std_msgs.msg import Bool
    rospy.init_node('wait_official_sensor', anonymous=True, argv=['wait_official_sensor'], disable_signals=True)
    # The original TSP solver writes these files before any path is generated.
    tsp_dir = Path.home()/'.ros'/'fuel_gazebo'/'tsp'
    tsp_dir.mkdir(parents=True, exist_ok=True)
    received = {'ready': False, 'at': 0.}
    def callback(msg):
        received.update(ready=msg.data, at=time.monotonic())
    subscriber = rospy.Subscriber('/fuel_official/sensor_ready', Bool, callback, queue_size=1)
    deadline = time.monotonic()+45.
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if (received['ready'] and time.monotonic()-received['at'] < .5 and
                rospy.get_param('/fuel_official/sensor_configured', False)):
            subscriber.unregister()
            rospy.signal_shutdown('Starting original FUEL executable with measured calibration')
            os.execv(sys.argv[1], sys.argv[1:])
        time.sleep(.05)
    raise SystemExit('Official camera data/calibration not ready within 45 wall seconds; C++ node not started.')


if __name__ == '__main__':
    main()
