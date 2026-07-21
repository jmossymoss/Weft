#!/usr/bin/env python3
"""Stratified sampler for the ABC Dataset broad-nightly public corpus.

Selects a small set of STEP files from a local ABC tree (WEFT_ABC_ROOT),
copies them into tests/public_corpus/_cache/abc/, and rewrites
tests/public_corpus/abc_nightly.tsv.

Stratification (cheap, no full meshing):
  1. Prefer sibling ABC stats YAML ``#surfs`` (B-rep surface count).
  2. Else count ADVANCED_FACE / FACE_SURFACE entities in the STEP text.
  3. Else fall back to file-size bands as a face-count proxy.

Usage:
  export WEFT_ABC_ROOT=/path/to/abc   # unpacked step/ tree (or any STEP tree)
  tools/sample_abc_nightly.py
  # or:
  tools/fetch_public_corpus.sh abc-nightly

Does not download multi-GB ABC STEP chunks. Does not commit STEP blobs.
"""

from __future__ import annotations

import argparse
import os
import random
import re
import shutil
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CACHE = Path(
    os.environ.get(
        "WEFT_PUBLIC_CORPUS_ROOT",
        str(REPO / "tests" / "public_corpus" / "_cache"),
    )
)
DEFAULT_MANIFEST = REPO / "tests" / "public_corpus" / "abc_nightly.tsv"
DEFAULT_SEED = 42

# (band_id, lo_inclusive, hi_inclusive) — B-rep surface / face proxy bands.
FACE_BANDS = (
    ("1-100", 1, 100),
    ("100-500", 101, 500),
    ("500-2000", 501, 2000),
)

# File-size proxy (bytes) when entity counts are unavailable.
SIZE_BANDS = (
    ("1-100", 1, 80_000),
    ("100-500", 80_001, 600_000),
    ("500-2000", 600_001, 8_000_000),
)

# Stable nightly smoke slots: one primary per band + optional extras.
PRIMARY_SLOTS = ["1-100", "100-500", "500-2000"]
EXTRA_PER_BAND = 1  # total up to 6 when inventory allows
MAX_CASES = 6

SURFS_RE = re.compile(r"^['\"]?#surfs['\"]?\s*:\s*(\d+)\s*$", re.MULTILINE)
ADVANCED_FACE_RE = re.compile(r"\bADVANCED_FACE\b", re.IGNORECASE)
FACE_SURFACE_RE = re.compile(r"\bFACE_SURFACE\b", re.IGNORECASE)
STEP_SUFFIXES = {".step", ".stp", ".STEP", ".STP"}


def die(msg: str, code: int = 1) -> None:
    print(f"sample_abc_nightly: {msg}", file=sys.stderr)
    raise SystemExit(code)


def find_step_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            p = Path(dirpath) / name
            if p.suffix in STEP_SUFFIXES:
                files.append(p)
    files.sort()
    return files


def sibling_stats_yml(step: Path) -> Path | None:
    """Locate an ABC stats YAML near a STEP file when present."""
    stem = step.stem
    # Common: 00000002_..._step_001.step → 00000002_..._stats_001.yml
    candidates: list[Path] = []
    for parent in (step.parent, step.parent.parent):
        if not parent.is_dir():
            continue
        for p in parent.glob("*.yml"):
            if "stats" in p.name.lower() or "stat" in p.name.lower():
                # Prefer same model id prefix (first token before '_')
                if stem.split("_")[0] in p.name:
                    return p
                candidates.append(p)
        # Alternate naming: same stem with stats
        alt = parent / (stem.replace("_step_", "_stats_") + ".yml")
        if alt.is_file():
            return alt
        alt2 = parent / (stem + "_stats.yml")
        if alt2.is_file():
            return alt2
    return candidates[0] if candidates else None


def surfs_from_stats(yml: Path) -> int | None:
    try:
        text = yml.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    m = SURFS_RE.search(text)
    if m:
        return int(m.group(1))
    # Fallback: length of surfs: [...] list is expensive; skip.
    return None


def face_proxy_from_step(step: Path, max_bytes: int = 12_000_000) -> int | None:
    """Cheap B-rep face proxy: count ADVANCED_FACE (else FACE_SURFACE)."""
    try:
        size = step.stat().st_size
        if size > max_bytes:
            return None
        # Read in chunks; entity names appear in ASCII STEP text.
        data = step.read_bytes()
    except OSError:
        return None
    # Decode latin-1 to preserve bytes; STEP is ASCII-heavy.
    text = data.decode("latin-1", errors="ignore")
    n = len(ADVANCED_FACE_RE.findall(text))
    if n > 0:
        return n
    n = len(FACE_SURFACE_RE.findall(text))
    return n if n > 0 else None


def size_band(size: int) -> str | None:
    for band, lo, hi in SIZE_BANDS:
        if lo <= size <= hi:
            return band
    return None


def face_band(nfaces: int) -> str | None:
    for band, lo, hi in FACE_BANDS:
        if lo <= nfaces <= hi:
            return band
    return None


def classify(step: Path) -> tuple[str, int, str] | None:
    """Return (band, proxy_value, method) or None if out of range."""
    stats = sibling_stats_yml(step)
    if stats is not None:
        surfs = surfs_from_stats(stats)
        if surfs is not None and surfs > 0:
            band = face_band(surfs)
            if band:
                return band, surfs, "stats_surfs"
    nfaces = face_proxy_from_step(step)
    if nfaces is not None:
        band = face_band(nfaces)
        if band:
            return band, nfaces, "step_entities"
    try:
        size = step.stat().st_size
    except OSError:
        return None
    band = size_band(size)
    if band:
        return band, size, "filesize"
    return None


def stable_id(band: str, step: Path, index: int) -> str:
    tag = {"1-100": "S", "100-500": "M", "500-2000": "L"}.get(band, "X")
    # Prefer ABC numeric folder / stem prefix when present.
    prefix = step.stem.split("_")[0]
    if not prefix.isdigit():
        prefix = f"{index:04d}"
    return f"ABC_NIGHTLY_{tag}{index}_{prefix}"


def rel_cache_path(step: Path, band: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", step.name)
    band_dir = band.replace("-", "_")
    return f"abc/nightly/band_{band_dir}/{safe}"


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    lines = [
        "# ABC Dataset broad-nightly / geometric-diversity sample.",
        "# Paths are relative to tests/public_corpus/_cache/",
        "# Generated by tools/sample_abc_nightly.py — re-run after updating WEFT_ABC_ROOT.",
        "# Upstream: https://deep-geometry.github.io/abc-dataset/",
        "# id\tface_band\tlocal_path\tvalidity\tnotes",
    ]
    for r in rows:
        lines.append(
            "\t".join(
                [
                    r["id"],
                    r["face_band"],
                    r["local_path"],
                    r["validity"],
                    r["notes"],
                ]
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_smoke_skeleton(path: Path) -> None:
    """Expected relative paths when ABC STEP is not yet sampled locally."""
    rows = [
        {
            "id": "ABC_NIGHTLY_S1",
            "face_band": "1-100",
            "local_path": "abc/nightly/band_1_100/ABC_NIGHTLY_S1.step",
            "validity": "closed_solid",
            "notes": "expected_slot; run tools/sample_abc_nightly.py with WEFT_ABC_ROOT",
        },
        {
            "id": "ABC_NIGHTLY_M1",
            "face_band": "100-500",
            "local_path": "abc/nightly/band_100_500/ABC_NIGHTLY_M1.step",
            "validity": "closed_solid",
            "notes": "expected_slot; run tools/sample_abc_nightly.py with WEFT_ABC_ROOT",
        },
        {
            "id": "ABC_NIGHTLY_L1",
            "face_band": "500-2000",
            "local_path": "abc/nightly/band_500_2000/ABC_NIGHTLY_L1.step",
            "validity": "closed_solid",
            "notes": "expected_slot; run tools/sample_abc_nightly.py with WEFT_ABC_ROOT",
        },
    ]
    write_manifest(path, rows)


def sample(
    abc_root: Path,
    cache: Path,
    manifest: Path,
    seed: int,
    max_scan: int,
) -> int:
    steps = find_step_files(abc_root)
    if not steps:
        die(f"no STEP files under {abc_root}")

    rng = random.Random(seed)
    if len(steps) > max_scan:
        steps = rng.sample(steps, max_scan)
        steps.sort()

    by_band: dict[str, list[tuple[Path, int, str]]] = defaultdict(list)
    for step in steps:
        c = classify(step)
        if c is None:
            continue
        band, value, method = c
        by_band[band].append((step, value, method))

    for band in by_band:
        by_band[band].sort(key=lambda t: (t[0].as_posix(), t[1]))

    dest_root = cache / "abc" / "nightly"
    if dest_root.exists():
        shutil.rmtree(dest_root)
    dest_root.mkdir(parents=True, exist_ok=True)

    selected: list[dict[str, str]] = []
    used: set[Path] = set()

    def pick_one(band: str, index: int) -> bool:
        pool = [t for t in by_band.get(band, []) if t[0] not in used]
        if not pool:
            return False
        # Deterministic pick: seeded shuffle of band pool.
        pool = list(pool)
        rng.shuffle(pool)
        step, value, method = pool[0]
        used.add(step)
        rel = rel_cache_path(step, band)
        out = cache / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(step, out)
        cid = stable_id(band, step, index)
        selected.append(
            {
                "id": cid,
                "face_band": band,
                "local_path": rel,
                "validity": "closed_solid",
                "notes": f"proxy={value};method={method};src={step.name};seed={seed}",
            }
        )
        print(f"  + {cid}  {band}  {rel}  ({method}={value})")
        return True

    print(f"ABC root: {abc_root} ({len(steps)} STEP candidates scanned)")
    for band in PRIMARY_SLOTS:
        print(f"  band {band}: {len(by_band.get(band, []))} candidates")

    idx = 1
    for band in PRIMARY_SLOTS:
        if pick_one(band, idx):
            idx += 1

    # Optional extras per band for diversity (cap MAX_CASES).
    for _ in range(EXTRA_PER_BAND):
        for band in PRIMARY_SLOTS:
            if len(selected) >= MAX_CASES:
                break
            if pick_one(band, idx):
                idx += 1
        if len(selected) >= MAX_CASES:
            break

    if not selected:
        die(
            "no files matched face/size bands; check WEFT_ABC_ROOT contents "
            f"(scanned {len(steps)} STEP files)"
        )

    write_manifest(manifest, selected)
    status = {
        "seed": seed,
        "abc_root": str(abc_root),
        "scanned": len(steps),
        "selected": len(selected),
        "bands": {b: len(by_band.get(b, [])) for b in PRIMARY_SLOTS},
    }
    status_path = cache / "abc" / "nightly_status.json"
    status_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_text(
        __import__("json").dumps(status, indent=2) + "\n", encoding="utf-8"
    )
    print(f"wrote {manifest} ({len(selected)} rows)")
    print(f"cache: {dest_root}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--root",
        type=Path,
        default=None,
        help="ABC STEP tree (default: WEFT_ABC_ROOT)",
    )
    ap.add_argument(
        "--cache",
        type=Path,
        default=DEFAULT_CACHE,
        help="public corpus cache root",
    )
    ap.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_MANIFEST,
        help="output TSV path",
    )
    ap.add_argument("--seed", type=int, default=int(os.environ.get("WEFT_ABC_SEED", DEFAULT_SEED)))
    ap.add_argument(
        "--max-scan",
        type=int,
        default=int(os.environ.get("WEFT_ABC_MAX_SCAN", "4000")),
        help="max STEP files to classify (random subset if larger)",
    )
    ap.add_argument(
        "--skeleton-only",
        action="store_true",
        help="rewrite manifest expected-slot skeleton without sampling",
    )
    args = ap.parse_args()

    if args.skeleton_only:
        write_smoke_skeleton(args.manifest)
        print(f"wrote expected-slot skeleton: {args.manifest}")
        return 0

    root = args.root
    if root is None:
        env = os.environ.get("WEFT_ABC_ROOT", "").strip()
        if not env:
            die(
                "set WEFT_ABC_ROOT to a local ABC STEP tree, or pass --root\n"
                "  Upstream: https://deep-geometry.github.io/abc-dataset/\n"
                "  Chunk list: https://deep-geometry.github.io/abc-dataset/data/step_v00.txt\n"
                "  Then: tools/sample_abc_nightly.py"
            )
        root = Path(env)
    if not root.is_dir():
        die(f"ABC root is not a directory: {root}")

    return sample(root, args.cache, args.manifest, args.seed, args.max_scan)


if __name__ == "__main__":
    raise SystemExit(main())
