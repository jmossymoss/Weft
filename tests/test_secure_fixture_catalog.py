#!/usr/bin/env python3
"""Verify the frozen weftocct fixture catalogue and its committed evidence."""

from __future__ import annotations

import hashlib
import json
import sys
from collections import Counter
from pathlib import Path, PurePosixPath
from typing import Any


EXPECTED_KINDS = {
    "occt_procedural": 77,
    "derived_corruption": 21,
    "native_fault_injection": 5,
    "audited_external": 3,
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def fixture_path(root: Path, manifest_path: str) -> Path:
    relative = PurePosixPath(manifest_path)
    require(not relative.is_absolute(), f"absolute manifest path: {manifest_path}")
    parts = relative.parts
    if parts and parts[0] == "fixtures":
        parts = parts[1:]
    require(parts and ".." not in parts, f"unsafe manifest path: {manifest_path}")
    resolved = (root.joinpath(*parts)).resolve()
    require(root == resolved or root in resolved.parents,
            f"manifest path escapes snapshot: {manifest_path}")
    return resolved


def verify_bound_file(root: Path, document: dict[str, Any], path_key: str,
                      digest_key: str, owner: str) -> None:
    path_value = document.get(path_key)
    digest_value = document.get(digest_key)
    if path_value is None and digest_value is None:
        return
    require(type(path_value) is str and type(digest_value) is str,
            f"{owner}: incomplete {path_key}/{digest_key} binding")
    path = fixture_path(root, path_value)
    require(path.is_file(), f"{owner}: missing {path_value}")
    require(digest(path) == digest_value,
            f"{owner}: digest mismatch for {path_value}")


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        raise ValueError("usage: test_secure_fixture_catalog.py SNAPSHOT_ROOT")
    root = Path(argv[1]).resolve(strict=True)
    manifest_path = root / "manifest.json"
    sidecar = (root / "manifest.canonical.sha256").read_text("ascii").split()
    require(len(sidecar) == 2 and sidecar[1] == "manifest.json",
            "invalid canonical manifest sidecar")
    require(digest(manifest_path) == sidecar[0], "manifest digest mismatch")

    manifest = json.loads(manifest_path.read_text("utf-8"))
    fixtures = manifest.get("fixtures")
    require(type(fixtures) is list and len(fixtures) == 106,
            "catalog must contain exactly 106 fixtures")
    identifiers = [fixture.get("id") for fixture in fixtures]
    require(all(type(identifier) is str and identifier for identifier in identifiers),
            "every fixture needs a non-empty string id")
    require(len(set(identifiers)) == len(identifiers), "fixture ids are not unique")

    kinds = Counter(fixture.get("source", {}).get("kind") for fixture in fixtures)
    require(dict(kinds) == EXPECTED_KINDS,
            f"fixture lane counts changed: {dict(kinds)}")

    for fixture in fixtures:
        identifier = fixture["id"]
        verify_bound_file(root, fixture, "expectation_path", "expectation_sha256",
                          identifier)
        verify_bound_file(root, fixture, "review_record_path",
                          "review_record_sha256", identifier)
        source = fixture["source"]
        verify_bound_file(root, source, "artifact_path", "artifact_sha256",
                          identifier)
        verify_bound_file(root, source, "baseline_artifact_path",
                          "baseline_artifact_sha256", identifier)
        verify_bound_file(root, source, "transformation_source",
                          "transformation_source_sha256", identifier)
        licence = source.get("licence")
        if licence:
            verify_bound_file(root, licence, "evidence", "evidence_sha256",
                              identifier)
        provenance = source.get("provenance")
        if provenance:
            verify_bound_file(root, provenance, "audit_record",
                              "audit_record_sha256", identifier)
        for generator_source in source.get("generator_sources", []):
            require(fixture_path(root, generator_source).is_file(),
                    f"{identifier}: missing generator source {generator_source}")

    require(len(list((root / "expected").glob("*.json"))) == 106,
            "expected-oracle file count changed")
    require(len(list((root / "reviews").glob("*.json"))) == 106,
            "review-record file count changed")
    require((root / "UPSTREAM.md").is_file(), "snapshot provenance is missing")
    print("secure fixture catalogue checks passed "
          "(106 fixtures: 77 procedural, 21 derived, 5 native, 3 external)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"secure fixture catalogue check failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error
