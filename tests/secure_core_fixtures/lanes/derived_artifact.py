#!/usr/bin/env python3
"""Deterministic, source-bound mutations for derived STEP and ASCII BRep fixtures.

The manifest carries every exact replacement.  This module deliberately does not
infer entity numbers or silently broaden a match: a baseline drift therefore fails
before an adversarial artifact can be regenerated with different semantics.
"""

from __future__ import annotations

import hashlib
from typing import Any


TRANSFORMATION_CODE = "transform.declared_ascii_patch.v1"


class DerivationError(ValueError):
    """Raised when a declared mutation does not bind its exact baseline."""


def _replace_occurrence(text: str, old: str, new: str, occurrence: int) -> str:
    start = 0
    found = -1
    for _ in range(occurrence):
        found = text.find(old, start)
        if found < 0:
            raise DerivationError("declared mutation occurrence is absent")
        start = found + len(old)
    return text[:found] + new + text[found + len(old) :]


def apply_declared_ascii_patch(baseline: bytes, source: dict[str, Any]) -> bytes:
    """Apply the manifest's ordered exact replacements and return exact bytes."""

    if type(baseline) is not bytes:
        raise DerivationError("baseline must be exact bytes")
    if hashlib.sha256(baseline).hexdigest() != source["baseline_artifact_sha256"]:
        raise DerivationError("baseline bytes do not match the source-bound digest")
    if source["transformation_code"] != TRANSFORMATION_CODE:
        raise DerivationError("wrong deterministic transformation code")
    if source["artifact_format"] not in {"step", "occt_ascii_brep"}:
        raise DerivationError("unsupported derived artifact format")

    try:
        text = baseline.decode("ascii")
    except UnicodeDecodeError as error:
        raise DerivationError("derived artifact baseline must be canonical ASCII") from error

    parameters = source["transformation_parameters"]
    if not parameters["reason"].strip():
        raise DerivationError("derived mutation needs a non-empty reason")
    operations = parameters["operations"]
    if not operations:
        raise DerivationError("derived mutation needs at least one operation")

    for index, operation in enumerate(operations):
        if operation["kind"] != "replace_ascii_exact":
            raise DerivationError(f"operation {index} has an unknown kind")
        old = operation["old"]
        new = operation["new"]
        expected_matches = operation["expected_matches"]
        occurrence = operation["occurrence"]
        actual_matches = text.count(old)
        if actual_matches != expected_matches:
            raise DerivationError(
                f"operation {index} expected {expected_matches} exact matches, "
                f"found {actual_matches}"
            )
        if occurrence < 1 or occurrence > actual_matches:
            raise DerivationError(f"operation {index} occurrence is outside the match set")
        text = _replace_occurrence(text, old, new, occurrence)

    result = text.encode("ascii")
    if result == baseline:
        raise DerivationError("declared mutation did not change the baseline")
    return result
