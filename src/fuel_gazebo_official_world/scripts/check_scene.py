#!/usr/bin/env python3
"""Offline check of official PCD, quantization, merged geometry and startup clearance."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def package_root():
    path = Path(__file__).resolve().parents[1]
    if (path / 'manifest.json').is_file():
        return path
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path('fuel_gazebo_official_world'))
    except Exception as exc:
        raise RuntimeError('Cannot locate fuel_gazebo_official_world package.') from exc


def parse_pcd(path):
    import numpy as np
    with path.open() as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Missing PCD DATA header.')
            if line.startswith('#') or not line.strip():
                continue
            fields = line.strip().split()
            header[fields[0]] = fields[1:]
            if fields[0] == 'DATA':
                break
        if header.get('DATA') != ['ascii'] or header.get('FIELDS') != ['x', 'y', 'z']:
            raise ValueError('Expected official ASCII xyz PCD.')
        points = np.loadtxt(stream)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Invalid point array.')
    if len(points) != int(header['POINTS'][0]):
        raise ValueError('PCD point count differs from header.')
    return points


def check(root=None):
    import numpy as np
    root = Path(root) if root else package_root()
    manifest = json.loads((root / 'manifest.json').read_text())
    pcd = root / 'resource/office.pcd'
    sdf_path = root / 'models/fuel_official_office/model.sdf'
    source_sha = 'f81065db52a6d83ae6d9bb357f321e45327f59eb7105cd005d18e8a1ba3f2336'
    if manifest['source_sha256'] != source_sha or hashlib.sha256(pcd.read_bytes()).hexdigest() != source_sha:
        raise ValueError('Source office.pcd hash changed.')
    if hashlib.sha256(sdf_path.read_bytes()).hexdigest() != manifest['model_sdf_sha256']:
        raise ValueError('Generated model.sdf hash changed.')
    if manifest['source_commit'] != '662dd23c7b52b258d3c4a0155ff6632118e8984f':
        raise ValueError('Unexpected official source commit.')
    points = parse_pcd(pcd)
    if len(points) != manifest['source_points'] or not np.allclose(points.min(0), manifest['source_bounds_min'], atol=1e-8, rtol=0) or not np.allclose(points.max(0), manifest['source_bounds_max'], atol=1e-8, rtol=0):
        raise ValueError('Source dimensions changed.')
    resolution = manifest['resolution_m']
    if resolution != 0.1:
        raise ValueError('Unexpected grid resolution.')
    indices = np.floor(points / resolution + 0.5 + 1e-7).astype(np.int64)
    if np.any(np.abs(points - indices * resolution) > resolution / 2 + 1e-8):
        raise ValueError('A source point is outside its occupied cube.')
    cells = np.unique(indices, axis=0)
    lower = np.array(manifest['grid_min'], dtype=np.int64)
    upper = np.array(manifest['grid_max'], dtype=np.int64)
    if not np.array_equal(cells.min(0), lower) or not np.array_equal(cells.max(0), upper):
        raise ValueError('Grid bounds changed.')
    expected = np.zeros(tuple(upper - lower + 1), dtype=bool)
    expected[tuple((cells - lower).T)] = True
    coverage = np.zeros_like(expected, dtype=np.uint16)
    model = ET.parse(sdf_path).getroot().find('model')
    if model is None or model.attrib.get('name') != manifest['model_name'] or model.findtext('static') != 'true':
        raise ValueError('Model must have expected name and be static.')
    links = model.findall('link')
    if len(links) != 1 or links[0].attrib.get('name') != manifest['link_marker']:
        raise ValueError('Unexpected model link marker.')
    link = links[0]
    if link.find('pose') is not None or model.find('pose') is not None:
        raise ValueError('Unexpected transform on model or link.')
    if len(link.findall('collision')) != len(manifest['boxes']) or len(link.findall('visual')) != len(manifest['boxes']):
        raise ValueError('Every box requires one matching collision and visual.')
    physical_boxes = []
    for number, box in enumerate(manifest['boxes']):
        lo = np.array(box['min_inclusive'], dtype=np.int64)
        hi = np.array(box['max_exclusive'], dtype=np.int64)
        if np.any(hi <= lo) or np.any(lo < lower) or np.any(hi > upper + 1):
            raise ValueError('Invalid merged box extent.')
        coverage[tuple(slice(int(a), int(b)) for a, b in zip(lo - lower, hi - lower))] += 1
        center = (lo + hi - 1) * resolution / 2
        size = (hi - lo) * resolution
        for kind in ['collision', 'visual']:
            element = link.find(f"{kind}[@name='wall_{number:03d}_{kind}']")
            if element is None or element.find('geometry/box/size') is None:
                raise ValueError('Missing box geometry.')
            pose = np.fromstring(element.findtext('pose', ''), sep=' ')
            actual_size = np.fromstring(element.findtext('geometry/box/size', ''), sep=' ')
            if pose.shape != (6,) or actual_size.shape != (3,) or not np.allclose(pose, [*center, 0, 0, 0], atol=1e-8, rtol=0) or not np.allclose(actual_size, size, atol=1e-8, rtol=0):
                raise ValueError('Visual/collision dimensions or coordinates differ from source cells.')
        physical_boxes.append(((lo - 0.5) * resolution, (hi - 0.5) * resolution))
    if len(cells) != manifest['occupied_cells'] or len(manifest['boxes']) != manifest['box_count']:
        raise ValueError('Unexpected occupied cell or box count.')
    if not np.all(coverage[expected] == 1) or np.any(coverage[~expected] != 0):
        raise ValueError('Box merging adds, omits or overlaps occupied cells.')
    # Vertical segment from (0,0,1) to (0,0,2), including a 0.35 m sphere envelope.
    clear = manifest['clearance_test']
    if clear != {'segment_start': [0, 0, 1], 'segment_end': [0, 0, 2], 'margin_m': 0.35, 'ground_z': 0}:
        raise ValueError('Startup clearance definition changed.')
    start = np.array(clear['segment_start'], dtype=float)
    end = np.array(clear['segment_end'], dtype=float)
    if not np.array_equal(start[:2], end[:2]) or end[2] < start[2]:
        raise ValueError('Clearance check is defined for an upward vertical segment.')
    minimum = float('inf')
    for lo, hi in physical_boxes:
        delta = np.maximum(np.maximum(lo - end, start - hi), 0.0)
        minimum = min(minimum, float(np.linalg.norm(delta)))
    ground_clearance = float(start[2] - clear['ground_z'])
    if min(minimum, ground_clearance) <= clear['margin_m']:
        raise ValueError('Official startup segment lacks the required obstacle/ground margin.')
    return {'result': 'OFFICIAL OFFICE OFFLINE CHECK PASSED', 'source_points': len(points), 'occupied_cells': len(cells), 'merged_boxes': len(physical_boxes), 'resolution_m': resolution, 'minimum_wall_clearance_m': minimum, 'minimum_ground_clearance_m': ground_clearance, 'required_margin_m': clear['margin_m'], 'note': 'Offline geometry validation only; no Gazebo or flight test performed.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, help='Explicit package root (usually unnecessary).')
    args = parser.parse_args()
    try:
        result = check(args.root)
    except Exception as exc:
        print('OFFICIAL OFFICE CHECK FAILED: ' + str(exc), file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
