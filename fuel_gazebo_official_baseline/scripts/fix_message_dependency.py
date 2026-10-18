#!/usr/bin/env python3
"""Fix the pinned baseline's ROS message build order, preserving its manifest."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

COMMIT='662dd23c7b52b258d3c4a0155ff6632118e8984f'
RELATIVE='uav_simulator/Utils/multi_map_server/CMakeLists.txt'
OLD='add_dependencies(multi_map_visualization multi_map_server_messages_cpp)'
NEW='add_dependencies(multi_map_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})'
PATCH={
    'schema':1,'upstream_commit':COMMIT,
    'description':'Generate message headers before compiling the multi-map visualization node. No exploration, sensor or runtime changes.',
    'original_sha256':{RELATIVE:'fa4a79e5e3ead0624924c3bc47ca88364081f7ae263a1688e6f487ddc545e94b'},
    'patches':[{'path':RELATIVE,'old':OLD,'new':NEW,
                'reason':'The old dependency names a legacy target rather than the generated message targets; parallel compilation can start before the header exists.'}]
}
PATCH_BYTES=(json.dumps(PATCH,indent=2)+'\n').encode()


def sha(data):return hashlib.sha256(data).hexdigest()


def atomic(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=str(path.parent),delete=False) as f:
        temporary=Path(f.name)
        f.write(data)
    try:os.replace(str(temporary),str(path))
    finally:
        if temporary.exists():temporary.unlink()


def fix(workspace):
    workspace=workspace.expanduser().resolve()
    for forbidden in (Path.home()/'fuel_ws',Path.home()/'rotors_ws'):
        if workspace==forbidden.resolve() or forbidden.resolve() in workspace.parents:
            raise RuntimeError('Refusing to modify the existing flight workspaces')
    marker=workspace/'fuel_official_baseline_manifest.json'
    if not marker.is_file() or marker.is_symlink():raise RuntimeError('Not a recognized independent official workspace')
    marker_bytes=marker.read_bytes()
    record=json.loads(marker_bytes)
    if record.get('commit')!=COMMIT:raise RuntimeError('Unexpected official source version')
    source=workspace/'src/FUEL'
    expected=record['source_sha256']
    actual={}
    for p in source.rglob('*'):
        if p.is_symlink():raise RuntimeError('Unexpected source symlink: '+str(p))
        if p.is_file():actual[str(p.relative_to(source))]=sha(p.read_bytes())
    if actual!=expected:raise RuntimeError('Source differs from its preparation record; no files changed')
    target=source/RELATIVE
    before=target.read_bytes()
    patch=workspace/'src/fuel_gazebo_official_baseline/patches/build_order.json'
    if patch.is_symlink():raise RuntimeError('Unexpected patch symlink')
    previous_patch=patch.read_bytes() if patch.exists() else None
    if before.decode().count(NEW)==1:
        if previous_patch!=PATCH_BYTES or record['patch_sets'].get('build_order.json')!=sha(PATCH_BYTES):
            raise RuntimeError('The source is already changed but its patch record does not match')
        print('MESSAGE BUILD ORDER ALREADY FIXED')
        return
    if sha(before)!=PATCH['original_sha256'][RELATIVE] or before.decode().count(OLD)!=1:
        raise RuntimeError('Unexpected CMake contents; no files changed')
    if previous_patch is not None and previous_patch!=PATCH_BYTES:raise RuntimeError('A different build-order patch already exists')
    after=before.decode().replace(OLD,NEW,1).encode()
    record['source_sha256'][RELATIVE]=sha(after)
    record['patch_sets']['build_order.json']=sha(PATCH_BYTES)
    backup=workspace/'source_backups/message_build_order'
    backup.mkdir(parents=True,exist_ok=True)
    for name,data in [('CMakeLists.txt',before),('manifest.json',marker_bytes)]:
        file=backup/name
        if not file.exists():file.write_bytes(data)
        elif file.read_bytes()!=data:raise RuntimeError('An inconsistent earlier backup exists; no files changed')
    try:
        atomic(target,after)
        atomic(patch,PATCH_BYTES)
        atomic(marker,(json.dumps(record,sort_keys=True,indent=2)+'\n').encode())
    except Exception:
        atomic(target,before)
        atomic(marker,marker_bytes)
        if previous_patch is None:
            if patch.exists():patch.unlink()
        else:atomic(patch,previous_patch)
        raise
    print('MESSAGE BUILD ORDER FIXED')
    print('Workspace: '+str(workspace))
    print('Only the message-header build dependency changed; existing compiled results can be reused.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,default=Path.home()/'fuel_official_ws')
    args=p.parse_args()
    try:fix(args.workspace)
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        p.exit(1,'FIX STOPPED: '+str(exc)+'\n')


if __name__=='__main__':main()
