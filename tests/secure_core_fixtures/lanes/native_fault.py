#!/usr/bin/env python3
"""Adapter for deterministic native-fault fixture executables."""

from __future__ import annotations

import json
import hashlib
import subprocess
from pathlib import Path
from typing import Any, Callable


class NativeFaultError(RuntimeError):
    """Raised when a native fixture does not return its exact source-bound result."""


def executable_runner(executable: Path) -> Callable[[str, dict[str, Any]], dict[str, str]]:
    """Bind a built native harness to the catalog runner contract."""

    resolved = executable.resolve(strict=True)

    def run(fixture_id: str, source: dict[str, Any]) -> dict[str, str]:
        canonical_source = (
            json.dumps(source, ensure_ascii=True, indent=2, sort_keys=True) + "\n"
        ).encode("ascii")
        command = [
            str(resolved),
            "--fixture-id",
            fixture_id,
            "--fault-code",
            source["fault_code"],
            "--test-id",
            source["test_id"],
            "--harness-source-sha256",
            source["harness_source_sha256"],
            "--source-contract-sha256",
            hashlib.sha256(canonical_source).hexdigest(),
        ]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="strict",
        )
        if completed.returncode != 0:
            raise NativeFaultError(
                f"native fault harness failed with {completed.returncode}: "
                f"{completed.stderr.strip()}"
            )
        try:
            result = json.loads(completed.stdout)
        except json.JSONDecodeError as error:
            raise NativeFaultError("native fault harness returned invalid JSON") from error
        if type(result) is not dict or any(
            type(key) is not str or type(value) is not str
            for key, value in result.items()
        ):
            raise NativeFaultError("native fault harness result is not an exact string map")
        return result

    return run
