#!/usr/bin/env python3
"""Build a separate ideal-camera URDF without changing RotorS physics.

Python 3.8+, standard library only. This script writes a file; it never spawns,
deletes, moves, or changes the running Gazebo model or ROS parameters.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
REFERENCE = PACKAGE / "reference" / "hummingbird_vi_original.urdf"
BASE_LINK = "hummingbird/base_link"
ODOM_LINK = "hummingbird/odometry_sensor1_link"
SENSOR_LINK = "hummingbird/vi_sensor/camera_depth_link"
OPTICAL_LINK = "hummingbird/fuel_official_camera_optical_link"
OPTICAL_JOINT = "hummingbird/fuel_official_camera_optical_joint"
DEFAULT_OUTPUT = Path.home() / ".ros" / "fuel_official_baseline" / "robot.urdf"
WIDTH, HEIGHT = 640, 480
FOCAL_LENGTH = 387.229248046875
HORIZONTAL_FOV = 2.0 * math.atan(WIDTH / (2.0 * FOCAL_LENGTH))
NEAR_CLIP, FAR_CLIP = 0.35, 5.0
RATE = 15.0


def matmul(a, b):
    return [[sum(x * y for x, y in zip(row, col))
             for col in zip(*b)] for row in a]


def transpose(a):
    return [list(row) for row in zip(*a)]


def matvec(a, b):
    return [sum(x * y for x, y in zip(row, b)) for row in a]


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                            math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_rpy(r):
    p = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
    if abs(math.cos(p)) < 1e-10:
        raise ValueError("Unsupported near-singular camera installation")
    return [math.atan2(r[2][1], r[2][2]), p,
            math.atan2(r[1][0], r[0][0])]


def vec(text, length=3):
    values = [float(x) for x in text.split()]
    if len(values) != length or not all(math.isfinite(x) for x in values):
        raise ValueError("Invalid transform: " + text)
    return values


def fmt(values):
    return " ".join("{:.17g}".format(0.0 if abs(x) < 1e-15 else x)
                    for x in values)


def link_transform(root, ancestor, descendant):
    """Return the zero-joint-state ancestor<-descendant transform."""
    joints = {}
    for joint in root.findall("joint"):
        child, parent = joint.find("child"), joint.find("parent")
        if child is not None and parent is not None:
            if child.get("link") in joints:
                raise ValueError("URDF child has multiple parents")
            joints[child.get("link")] = joint
    rotation = rpy_matrix([0.0, 0.0, 0.0])
    translation = [0.0, 0.0, 0.0]
    name, visited = descendant, set()
    while name != ancestor:
        if name in visited or name not in joints:
            raise ValueError("No URDF path from {} to {}".format(ancestor, descendant))
        visited.add(name)
        joint = joints[name]
        # The odometry joint has zero limits; an unlocked moving installation
        # cannot be represented by a fixed sensor pose and is rejected.
        if joint.get("type") != "fixed":
            limit = joint.find("limit")
            if (limit is None or float(limit.get("lower", "nan")) != 0.0
                    or float(limit.get("upper", "nan")) != 0.0):
                raise ValueError("Moving camera/odometry installation is unsupported")
        origin = joint.find("origin")
        xyz = vec(origin.get("xyz", "0 0 0")) if origin is not None else [0.] * 3
        rpy = vec(origin.get("rpy", "0 0 0")) if origin is not None else [0.] * 3
        q = rpy_matrix(rpy)
        translation = [x + y for x, y in zip(xyz, matvec(q, translation))]
        rotation = matmul(q, rotation)
        name = joint.find("parent").get("link")
    return rotation, translation


def depth_sensor(root):
    matches = [(gazebo, sensor) for gazebo in root.findall("gazebo")
               for sensor in gazebo.findall("sensor") if sensor.get("type") == "depth"]
    if len(matches) != 1 or matches[0][0].get("reference") != SENSOR_LINK:
        raise ValueError("Expected exactly the captured Hummingbird depth sensor")
    return matches[0][1]


def signature(node):
    return (node.tag, tuple(sorted(node.attrib.items())), (node.text or "").strip(),
            tuple(signature(child) for child in node))


def physical_signature(root):
    """Everything except the explicitly replaced sensing-only elements."""
    root = copy.deepcopy(root)
    for child in list(root):
        if ((child.tag == "link" and child.get("name") == OPTICAL_LINK)
                or (child.tag == "joint" and child.get("name") == OPTICAL_JOINT)):
            root.remove(child)
    for gazebo in root.findall("gazebo"):
        for sensor in list(gazebo.findall("sensor")):
            if sensor.get("type") == "depth":
                gazebo.remove(sensor)
    return signature(root)


def set_text(parent, path, value):
    node = parent
    for name in path.split("/"):
        child = node.find(name)
        if child is None:
            child = ET.SubElement(node, name)
        node = child
    node.text = str(value)


def build_robot(source, reference):
    root = ET.fromstring(source)
    original = ET.fromstring(reference)
    if root.tag != "robot" or root.get("name") != "hummingbird":
        raise ValueError("This adapter is for the captured Hummingbird model only")
    if physical_signature(root) != physical_signature(original):
        raise ValueError("Robot physics or another sensor differs from the captured working model")
    sensor = depth_sensor(root)
    # Compute the sensor pose from its physical mounting link, rather than
    # mislabelling an image whose real optical center is still ahead of body.
    r_bs, t_bs = link_transform(root, BASE_LINK, SENSOR_LINK)
    r_bo, t_bo = link_transform(root, BASE_LINK, ODOM_LINK)
    r_so = matmul(transpose(r_bs), r_bo)
    t_so = matvec(transpose(r_bs), [a - b for a, b in zip(t_bo, t_bs)])
    set_text(sensor, "pose", fmt(t_so + matrix_rpy(r_so)))
    set_text(sensor, "update_rate", RATE)
    set_text(sensor, "camera/horizontal_fov", "{:.17g}".format(HORIZONTAL_FOV))
    set_text(sensor, "camera/image/width", WIDTH)
    set_text(sensor, "camera/image/height", HEIGHT)
    set_text(sensor, "camera/clip/near", NEAR_CLIP)
    set_text(sensor, "camera/clip/far", FAR_CLIP)
    plugins = sensor.findall("plugin")
    if (len(plugins) != 1
            or plugins[0].get("filename") != "libgazebo_ros_openni_kinect.so"):
        raise ValueError("Unexpected depth-camera ROS plugin")
    plugin = plugins[0]
    set_text(plugin, "frameName", OPTICAL_LINK)
    set_text(plugin, "updateRate", RATE)
    set_text(plugin, "pointCloudCutoff", NEAR_CLIP)
    set_text(plugin, "pointCloudCutoffMax", FAR_CLIP)
    # Eliminate a stale adapted link/joint before appending the verified pair.
    for child in list(root):
        if ((child.tag == "link" and child.get("name") == OPTICAL_LINK)
                or (child.tag == "joint" and child.get("name") == OPTICAL_JOINT)):
            root.remove(child)
    ET.SubElement(root, "link", {"name": OPTICAL_LINK})
    joint = ET.SubElement(root, "joint", {"name": OPTICAL_JOINT, "type": "fixed"})
    optical_rotation = rpy_matrix([-math.pi / 2, 0.0, -math.pi / 2])
    r_bopt = matmul(r_bo, optical_rotation)
    ET.SubElement(joint, "origin", {"xyz": fmt(t_bo), "rpy": fmt(matrix_rpy(r_bopt))})
    ET.SubElement(joint, "parent", {"link": BASE_LINK})
    ET.SubElement(joint, "child", {"link": OPTICAL_LINK})
    if physical_signature(root) != physical_signature(original):
        raise ValueError("Internal error: a protected physical element changed")
    # Check actual rendering pose against the optical TF, including rotation.
    actual_r = matmul(matmul(r_bs, rpy_matrix(matrix_rpy(r_so))), optical_rotation)
    actual_t = [a + b for a, b in zip(t_bs, matvec(r_bs, t_so))]
    tf_r, tf_t = link_transform(root, BASE_LINK, OPTICAL_LINK)
    error = max([abs(a - b) for a, b in zip(actual_t, tf_t)]
                + [abs(actual_r[i][j] - tf_r[i][j]) for i in range(3) for j in range(3)])
    if error > 1e-12:
        raise ValueError("Internal error: rendering pose and optical TF disagree")
    return root, {"body_to_original_sensor_translation": t_bs,
                  "sensor_relative_pose": t_so + matrix_rpy(r_so),
                  "body_to_new_optical_translation": tf_t,
                  "render_tf_max_error": error}


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=str(path.parent), prefix=path.name + ".",
                                     delete=False) as handle:
        temporary = Path(handle.name)
        handle.write(data)
    try:
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--input", type=Path, help="Original captured URDF (default: package reference)")
    group.add_argument("--from-running", action="store_true", help="Read /hummingbird/robot_description only")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        reference = REFERENCE.read_bytes()
        if args.from_running:
            import rospy
            rospy.init_node("make_fuel_official_robot", anonymous=True, disable_signals=True)
            source = rospy.get_param("/hummingbird/robot_description").encode("utf-8")
            source_name = "/hummingbird/robot_description (read only)"
        else:
            source_path = (args.input or REFERENCE).expanduser().resolve()
            source = source_path.read_bytes()
            source_name = str(source_path)
        output = args.output.expanduser().resolve()
        if output == REFERENCE.resolve() or (args.input and output == args.input.expanduser().resolve()):
            raise ValueError("Output must be a separate generated file")
        root, details = build_robot(source, reference)
        data = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
        details.update({"source": source_name,
                        "source_sha256": hashlib.sha256(source).hexdigest(),
                        "reference_sha256": hashlib.sha256(reference).hexdigest(),
                        "output_sha256": hashlib.sha256(data).hexdigest(),
                        "optical_frame": OPTICAL_LINK,
                        "width": WIDTH, "height": HEIGHT,
                        "horizontal_fov": HORIZONTAL_FOV,
                        "focal_length_from_fov": FOCAL_LENGTH,
                        "near_clip": NEAR_CLIP, "far_clip": FAR_CLIP,
                        "rate_hz": RATE, "protected_model_unchanged": True})
        atomic_write(output, data)
        atomic_write(output.with_suffix(".json"),
                     (json.dumps(details, indent=2) + "\n").encode("utf-8"))
        print("CAMERA MODEL CHECK PASSED")
        print("Generated: " + str(output))
        print("Original physical model: unchanged")
        print("Optical frame: " + OPTICAL_LINK)
        print("Use this file only when starting the separate simulation.launch.")
        return 0
    except (OSError, ValueError, KeyError, ET.ParseError, ImportError) as exc:
        print("CAMERA MODEL CHECK FAILED: " + str(exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
