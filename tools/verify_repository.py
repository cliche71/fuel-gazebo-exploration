#!/usr/bin/env python3
"""Verify delivered files and source provenance without importing ROS modules."""
import argparse
import ast
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys
import tarfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGES = (ROOT / "fuel_gazebo_official_baseline", ROOT / "fuel_gazebo_official_world")
RECORD = ROOT / "validation/delivery_file_hashes.json"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def delivery_files():
    result = {}
    for package in PACKAGES:
        if not package.is_dir():
            raise ValueError("Missing sibling package: " + str(package))
        for path in sorted(package.rglob("*")):
            if "__pycache__" in path.parts or path.suffix == ".pyc":
                raise ValueError("Generated Python cache in package: " + str(path))
            if path.is_symlink():
                raise ValueError("Unexpected symbolic link: " + str(path))
            if path.is_file():
                result[str(path.relative_to(ROOT))] = {
                    "sha256": sha256(path), "mode": oct(path.stat().st_mode & 0o777)
                }
    return result


def check_source():
    vendor = PACKAGES[0] / "vendor"
    manifest = json.loads((vendor / "upstream_manifest.json").read_text())
    reference = json.loads((vendor / "upstream_files.json").read_text())
    archive = vendor / manifest["archive"]
    if manifest["commit"] != reference["commit"] or sha256(archive) != manifest["sha256"]:
        raise ValueError("Pinned source archive does not match its manifest")
    expected = reference["files"]
    seen, names = set(), set()
    with tarfile.open(archive, "r:gz") as tar:
        for member in tar:
            relative = PurePosixPath(member.name)
            if relative.is_absolute() or ".." in relative.parts or member.name in names:
                raise ValueError("Unsafe or duplicate archive member: " + member.name)
            names.add(member.name)
            if member.isdir():
                continue
            if not member.isfile() or member.name not in expected:
                raise ValueError("Unexpected source member: " + member.name)
            stream = tar.extractfile(member)
            if stream is None:
                raise ValueError("Unreadable source member: " + member.name)
            data = stream.read()
            item = expected[member.name]
            blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
            if hashlib.sha256(data).hexdigest() != item["sha256"] or blob != item["git_blob"]:
                raise ValueError("Original source mismatch: " + member.name)
            seen.add(member.name)
    if seen != set(expected):
        raise ValueError("Original source file membership changed")
    return len(seen)


def check_links():
    documents = [ROOT / "README.md", ROOT / "SOURCE_MANIFEST.md"] + sorted((ROOT / "docs").glob("*.md"))
    count = 0
    for document in documents:
        for target in re.findall(r"\[[^\]]*\]\(([^\s)]+)\)", document.read_text()):
            if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target) or target.startswith("#"):
                continue
            path = target.split("#", 1)[0]
            if not (document.parent / path).exists():
                raise ValueError("Broken local link in " + str(document.relative_to(ROOT)) + ": " + target)
            count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh-delivery-record", action="store_true",
                        help="Record deliberate package changes before checking; does not validate flight.")
    args = parser.parse_args()
    actual = delivery_files()
    counts = {"python": 0, "json": 0, "xml": 0}
    for path in ROOT.rglob("*"):
        if not path.is_file() or ".git" in path.parts or "__pycache__" in path.parts:
            continue
        if path.suffix == ".py":
            ast.parse(path.read_text(), filename=str(path), feature_version=8)
            counts["python"] += 1
        elif path.suffix == ".json":
            json.loads(path.read_text())
            counts["json"] += 1
        elif path.suffix in {".xml", ".launch", ".world", ".sdf", ".urdf", ".config"}:
            ET.parse(path)
            counts["xml"] += 1
    version = (ROOT / "VERSION").read_text().strip()
    if ET.parse(PACKAGES[0] / "package.xml").getroot().findtext("version") != version:
        raise ValueError("VERSION differs from adapter package version")
    upstream_count = check_source()
    links = check_links()
    if args.refresh_delivery_record:
        RECORD.write_text(json.dumps({"schema": 1, "version": version, "files": actual},
                                     indent=2, sort_keys=True) + "\n")
    recorded = json.loads(RECORD.read_text())
    if recorded.get("version") != version or recorded.get("files") != actual:
        raise ValueError("Package contents differ from delivery_file_hashes.json; review deliberate changes first")
    print("REPOSITORY CHECK PASSED")
    print("Package files: %d; pinned official files: %d; local links: %d" %
          (len(actual), upstream_count, links))
    print("Syntax: " + json.dumps(counts, sort_keys=True))
    print("No ROS build or Gazebo flight was performed by this check.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, SyntaxError, ET.ParseError, tarfile.TarError) as error:
        print("REPOSITORY CHECK FAILED: " + str(error), file=sys.stderr)
        sys.exit(1)
