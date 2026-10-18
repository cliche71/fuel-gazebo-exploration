#!/usr/bin/env python3
"""Install the official office in an existing Gazebo hover session. Sends no flight commands."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

MODEL = 'fuel_official_office'
ALLOWED_MODELS = {'ground_plane', 'sun', 'hummingbird', 'depth_test_box', MODEL}
PLANNER_PATTERN = re.compile(r'(^|/)(planner|exploration_node|exploration_manager|fast_planner_node)(/|$)')


def field(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def success(response, action):
    if not field(response, 'success', False):
        raise RuntimeError(action + ': ' + str(field(response, 'status_message', 'Gazebo rejected request.')))
    return response


def graph_is_safe(names):
    blocked = sorted(name for name in names if PLANNER_PATTERN.search(name) or name.startswith('/fuel_exploration/') or name.startswith('/fuel_navigation/'))
    if blocked:
        raise RuntimeError('仍有导航或探索节点运行，请先在对应 roslaunch 终端按 Ctrl+C，只保留 Gazebo 与悬停控制：' + ', '.join(blocked))


def world_is_safe(names):
    unexpected = sorted(set(names) - ALLOWED_MODELS)
    if unexpected:
        raise RuntimeError('检测到未经本场景检查的模型，未改动 Gazebo：' + ', '.join(unexpected))
    if 'hummingbird' not in names or 'ground_plane' not in names:
        raise RuntimeError('仅用于已运行 hummingbird 和 ground_plane 的 Gazebo 悬停场景。预览请使用 world.launch。')


def model_matches(properties, state, manifest):
    success(properties, 'Read installed office properties')
    success(state, 'Read installed office pose')
    bodies = [name.split('::')[-1] for name in field(properties, 'body_names', [])]
    geometries = {name.split('::')[-1] for name in field(properties, 'geom_names', [])}
    expected = {f'wall_{i:03d}_collision' for i in range(manifest['box_count'])}
    if not field(properties, 'is_static', False) or bodies != [manifest['link_marker']] or geometries != expected:
        raise RuntimeError('已存在同名 office，但其标识或碰撞体不同；未覆盖或删除它。')
    pose = field(state, 'pose')
    position, orientation = field(pose, 'position'), field(pose, 'orientation')
    values = [field(position, axis) for axis in 'xyz'] + [field(orientation, axis) for axis in 'xyzw']
    if any(value is None or not math.isfinite(value) for value in values) or max(abs(x) for x in values[:6]) > 1e-6 or abs(abs(values[6]) - 1) > 1e-6:
        raise RuntimeError('已有 office 被移动或旋转，未继续修改场景。')


def hover_is_safe(state):
    success(state, 'Read hummingbird state')
    pose, twist = field(state, 'pose'), field(state, 'twist')
    position = field(pose, 'position')
    xyz = [field(position, axis) for axis in 'xyz']
    linear, angular = field(twist, 'linear'), field(twist, 'angular')
    v = [field(linear, axis) for axis in 'xyz']
    w = [field(angular, axis) for axis in 'xyz']
    if any(value is None or not math.isfinite(value) for value in xyz + v + w):
        raise RuntimeError('无法确认无人机位置或速度，未改动场景。')
    if math.hypot(xyz[0], xyz[1]) > 0.25 or not 0.6 <= xyz[2] <= 2.0 or sum(x*x for x in v) > 0.15**2 or sum(x*x for x in w) > 0.3**2:
        raise RuntimeError('请让无人机回到 (0,0,1) 附近稳定悬停后重试；当前场景未改动。')


def install_scene(api, manifest, sdf):
    """Dependency-injected operation; tested offline with a fake Gazebo API."""
    models = api.models()
    world_is_safe(models)
    graph_is_safe(api.nodes())
    hover_is_safe(api.state('hummingbird'))
    already = MODEL in models
    owned = False
    try:
        if already:
            model_matches(api.properties(MODEL), api.state(MODEL), manifest)
        else:
            success(api.spawn(MODEL, sdf), 'Spawn official office')
            owned = True
            model_matches(api.properties(MODEL), api.state(MODEL), manifest)
        # Verify again before changing the old scene; planner launch during installation must stop us.
        current = api.models()
        world_is_safe(current)
        graph_is_safe(api.nodes())
        hover_is_safe(api.state('hummingbird'))
        removed_box = False
        if 'depth_test_box' in current:
            success(api.delete('depth_test_box'), 'Remove old depth_test_box')
            removed_box = True
        return {'result': 'OFFICIAL OFFICE INSTALLED', 'already_installed': already, 'removed_depth_test_box': removed_box, 'note': 'Ground and robot unchanged; no flight or exploration command sent.'}
    except Exception as original:
        if owned:
            try:
                success(api.delete(MODEL), 'Rollback newly spawned office')
            except Exception as cleanup:
                raise RuntimeError(str(original) + '\n撤销新 office 失败，请保留悬停并检查 Gazebo：' + str(cleanup)) from original
        raise


class RosGazebo:
    def __init__(self, timeout):
        import rospy
        import rosnode
        from gazebo_msgs.srv import GetWorldProperties, GetModelProperties, GetModelState, SpawnModel, DeleteModel
        from geometry_msgs.msg import Pose
        self.rospy, self.rosnode, self.Pose = rospy, rosnode, Pose
        services = {'world': ('/gazebo/get_world_properties', GetWorldProperties), 'properties': ('/gazebo/get_model_properties', GetModelProperties), 'state': ('/gazebo/get_model_state', GetModelState), 'spawn': ('/gazebo/spawn_sdf_model', SpawnModel), 'delete': ('/gazebo/delete_model', DeleteModel)}
        self.calls = {}
        for key, (name, kind) in services.items():
            rospy.wait_for_service(name, timeout=timeout)
            self.calls[key] = rospy.ServiceProxy(name, kind)

    def models(self):
        response = success(self.calls['world'](), 'Read Gazebo models')
        return list(response.model_names)

    def nodes(self):
        return self.rosnode.get_node_names()

    def properties(self, name):
        return self.calls['properties'](name)

    def state(self, name):
        return self.calls['state'](name, 'world')

    def spawn(self, name, sdf):
        pose = self.Pose()
        pose.orientation.w = 1.0
        return self.calls['spawn'](name, sdf, '', pose, 'world')

    def delete(self, name):
        return self.calls['delete'](name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=8.0, help='Wall-clock seconds to wait for each Gazebo service.')
    args, unknown = parser.parse_known_args()
    if any(':=' not in arg for arg in unknown):
        parser.error('Unexpected arguments: ' + ' '.join(unknown))
    if not math.isfinite(args.timeout) or args.timeout <= 0 or args.timeout > 60:
        parser.error('--timeout must be within (0,60].')
    try:
        from check_scene import package_root, check
        root = package_root()
        check(root)
        manifest = json.loads((root / 'manifest.json').read_text())
        sdf = (root / 'models/fuel_official_office/model.sdf').read_text()
        import rospy
        rospy.init_node('spawn_fuel_official_office', anonymous=True)
        result = install_scene(RosGazebo(args.timeout), manifest, sdf)
        print(json.dumps(result, indent=2))
        return 0
    except Exception as exc:
        print('OFFICIAL OFFICE INSTALL FAILED: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
