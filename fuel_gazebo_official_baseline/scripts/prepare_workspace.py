#!/usr/bin/env python3
"""Prepare a separate, pinned FUEL workspace; never copy a modified worktree."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile


COMMIT = "662dd23c7b52b258d3c4a0155ff6632118e8984f"
UPSTREAM = "https://github.com/HKUST-Aerial-Robotics/FUEL.git"
PACKAGE_NAME = "fuel_gazebo_official_baseline"
MARKER_NAME = "fuel_official_baseline_manifest.json"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def below(path, parent):
    return path == parent or parent in path.parents


def run(arguments, **kwargs):
    return subprocess.check_output(arguments, stderr=subprocess.PIPE, **kwargs)


def safe_extract(archive, destination):
    """No links, duplicate paths, device files, or extraction outside the tree."""
    seen = set()
    with tarfile.open(archive, "r:*") as tar:
        for member in tar.getmembers():
            relative = Path(member.name)
            if relative.is_absolute() or ".." in relative.parts or member.name in seen:
                raise RuntimeError("Unsafe or duplicate archive member: " + member.name)
            seen.add(member.name)
            if not (member.isdir() or member.isfile()):
                raise RuntimeError("Unsupported archive member: " + member.name)
            target = destination / relative
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                stream = tar.extractfile(member)
                if stream is None:
                    raise RuntimeError("Unreadable archive member: " + member.name)
                with stream, target.open("wb") as output:
                    shutil.copyfileobj(stream, output)
                target.chmod(0o755 if member.mode & 0o111 else 0o644)


def verify_pristine(source, reference):
    expected = reference["files"]
    actual = {str(p.relative_to(source)) for p in source.rglob("*") if p.is_file()}
    if actual != set(expected):
        raise RuntimeError("Archive file list does not match the fixed official commit.")
    for relative, info in expected.items():
        path = source / relative
        if path.is_symlink():
            raise RuntimeError("Unexpected symbolic link: " + relative)
        data = path.read_bytes()
        blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        if digest(data) != info["sha256"] or blob != info["git_blob"]:
            raise RuntimeError("Official source mismatch: " + relative)


def obtain_archive(package, source_repo, reference, temporary):
    bundled = package / "vendor" / ("FUEL-" + COMMIT[:7] + ".tar.gz")
    if bundled.is_file():
        metadata_path = package / "vendor" / "upstream_manifest.json"
        metadata = json.loads(metadata_path.read_text())
        if (metadata.get("commit") != COMMIT or metadata.get("archive") != bundled.name or
                digest(bundled.read_bytes()) != metadata.get("sha256")):
            raise RuntimeError("Bundled official source archive verification failed.")
        return bundled, "bundled fixed official snapshot"

    archive = temporary / "official.tar"
    repositories = []
    if source_repo.is_dir():
        repositories.append(source_repo)
    for repository in repositories:
        try:
            tree = run(["git", "-C", str(repository), "rev-parse", COMMIT + "^{tree}"],
                       text=True, timeout=15).strip()
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
            continue
        if tree != reference["git_tree"]:
            raise RuntimeError("Local Git commit tree differs from the fixed official tree.")
        with archive.open("wb") as output:
            subprocess.run(["git", "-C", str(repository), "archive", "--format=tar", COMMIT, *reference["archive_paths"]],
                           stdout=output, stderr=subprocess.PIPE, check=True, timeout=60)
        return archive, "fixed Git objects from " + str(repository)

    if not shutil.which("git"):
        raise RuntimeError("No bundled archive or suitable local Git objects, and Git is unavailable.")
    repository = temporary / "upstream.git"
    subprocess.run(["git", "clone", "--bare", UPSTREAM, str(repository)],
                   check=True, timeout=180)
    tree = run(["git", "-C", str(repository), "rev-parse", COMMIT + "^{tree}"],
               text=True, timeout=15).strip()
    if tree != reference["git_tree"]:
        raise RuntimeError("Downloaded commit tree differs from the fixed official tree.")
    with archive.open("wb") as output:
        subprocess.run(["git", "-C", str(repository), "archive", "--format=tar", COMMIT, *reference["archive_paths"]],
                       stdout=output, stderr=subprocess.PIPE, check=True, timeout=60)
    return archive, "official Git archive at fixed commit"


def patch_source(source, package, backup):
    manifests = []
    pristine = {}
    for path in sorted((package / "patches").glob("*.json")):
        manifest = json.loads(path.read_text())
        if not isinstance(manifest, dict) or "patches" not in manifest:
            continue
        if manifest.get("upstream_commit", COMMIT) != COMMIT:
            raise RuntimeError("Patch set expects a different official commit: " + path.name)
        manifests.append((path.name, manifest))
        for relative, expected in manifest.get("original_sha256", {}).items():
            data = (source / relative).read_bytes()
            if digest(data) != expected:
                raise RuntimeError("Patch original mismatch: " + relative)

    # Validate every replacement in memory before writing any patched source.
    staged = {}
    for filename, manifest in manifests:
        for patch in manifest["patches"]:
            relative = patch["path"]
            if Path(relative).is_absolute() or ".." in Path(relative).parts:
                raise RuntimeError("Unsafe patch path in " + filename)
            if relative not in staged:
                pristine[relative] = (source / relative).read_bytes()
                staged[relative] = pristine[relative].decode("utf-8")
            old, new = patch["old"], patch["new"]
            if not old or staged[relative].count(old) != 1:
                raise RuntimeError("Patch context is not unique: " + filename + ": " + relative)
            staged[relative] = staged[relative].replace(old, new, 1)
    for relative, contents in staged.items():
        original = backup / relative
        original.parent.mkdir(parents=True, exist_ok=True)
        original.write_bytes(pristine[relative])
        (source / relative).write_text(contents, encoding="utf-8")
    return {name: digest((package / "patches" / name).read_bytes()) for name, _ in manifests}


def file_hashes(root):
    result = {}
    for path in root.rglob("*"):
        if path.is_symlink():
            raise RuntimeError("Unexpected link in prepared directory: " + str(path))
        if path.is_file():
            result[str(path.relative_to(root))] = digest(path.read_bytes())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.home() / "fuel_official_ws")
    parser.add_argument("--source-repo", type=Path, default=Path.home() / "fuel_ws/src/FUEL")
    parser.add_argument("--verify-only", action="store_true",
                        help="Verify an already prepared workspace without changing anything.")
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    workspace = args.workspace.expanduser().resolve()
    source_repo = args.source_repo.expanduser().resolve()
    for forbidden in (Path.home() / "fuel_ws", Path.home() / "rotors_ws"):
        if below(workspace, forbidden.resolve()):
            raise RuntimeError("The existing flight workspaces cannot be used: " + str(workspace))
    if workspace == Path(workspace.anchor) or workspace == Path.home().resolve():
        raise RuntimeError("Choose a dedicated new workspace directory.")
    if below(workspace, source_repo):
        raise RuntimeError("A new workspace cannot be created inside the source repository.")
    reference = json.loads((package / "vendor/upstream_files.json").read_text())
    if reference.get("commit") != COMMIT:
        raise RuntimeError("Official source reference expects a different commit.")

    marker = workspace / MARKER_NAME
    if marker.is_file():
        previous = json.loads(marker.read_text())
        if previous.get("commit") != COMMIT:
            raise RuntimeError("Prepared workspace belongs to a different official version.")
        if previous.get("source_sha256") != file_hashes(workspace / "src/FUEL"):
            raise RuntimeError("Prepared FUEL source has changed. Choose a new workspace; no files were overwritten.")
        patch_hashes = {p.name: digest(p.read_bytes()) for p in sorted((package / "patches").glob("*.json"))
                        if isinstance(json.loads(p.read_text()), dict) and "patches" in json.loads(p.read_text())}
        if previous.get("patch_sets") != patch_hashes:
            raise RuntimeError("Patch sets differ from this workspace. Choose a new workspace.")
        print("OFFICIAL WORKSPACE VERIFIED: " + str(workspace))
        print("Official commit: " + COMMIT)
        print("No files changed. Build and launch are separate commands.")
        return
    if args.verify_only:
        raise RuntimeError("No preparation record was found at " + str(workspace))
    if workspace.exists() and any(workspace.iterdir()):
        raise RuntimeError("Refusing to overwrite a nonempty directory: " + str(workspace))
    workspace.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="fuel_official_prepare_", dir=workspace.parent) as tmp:
        temporary = Path(tmp)
        archive, provenance = obtain_archive(package, source_repo, reference, temporary)
        prepared = temporary / "workspace"
        source = prepared / "src/FUEL"
        source.mkdir(parents=True)
        safe_extract(archive, source)
        verify_pristine(source, reference)
        patches = patch_source(source, package, prepared / "source_backups" / ("FUEL-" + COMMIT[:7]))
        # The adapter is copied separately from the verified upstream tree.
        shutil.copytree(package, prepared / "src" / PACKAGE_NAME,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        world_package = package.parent / 'fuel_gazebo_official_world'
        if not (world_package / 'manifest.json').is_file():
            raise RuntimeError('The sibling official world package is missing; extract the complete delivery bundle.')
        subprocess.run([sys.executable, str(world_package/'scripts/check_scene.py')], check=True)
        shutil.copytree(world_package, prepared / 'src' / 'fuel_gazebo_official_world',
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        report = {"schema": 1, "commit": COMMIT, "git_tree": reference["git_tree"],
                  "source": provenance, "patch_sets": patches, "source_sha256": file_hashes(source)}
        (prepared / MARKER_NAME).write_text(json.dumps(report, sort_keys=True, indent=2) + "\n")
        if workspace.exists():
            workspace.rmdir()  # The emptiness check above is repeated by rmdir itself.
        os.replace(prepared, workspace)
    print("OFFICIAL WORKSPACE PREPARED: " + str(workspace))
    print("Official commit: " + COMMIT)
    print("Verified official files: " + str(len(reference["files"])))
    print("Compatibility patch sets: " + ", ".join(sorted(patches)))
    print("Existing fuel_ws and rotors_ws were not changed.")
    print("No build or ROS launch was started.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print("PREPARATION STOPPED: " + str(error), file=sys.stderr)
        sys.exit(1)
