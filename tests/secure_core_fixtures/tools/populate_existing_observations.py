#!/usr/bin/env python3
"""Lift the existing independent v1 oracles into the strict observation model.

This migration consumes only reviewed expectation data and manifest construction
metadata.  It never reads generated summaries or production classifier output.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
EXPECTED_DIR = ROOT / "fixtures" / "expected"
MANIFEST_PATH = ROOT / "fixtures" / "manifest.json"
PROFILES_PATH = ROOT / "fixtures" / "obligation-profiles.json"

PROFILE_TO_KIND = {
    "continuity": "continuity_residual",
    "curve_geometry": "geometry",
    "curve_representation": "geometry",
    "evaluator_result": "pathology_measurement",
    "external_provenance": "source_import_normalization",
    "gate_result": "consumer_evidence",
    "locations": "transform",
    "numerical_measurement": "pathology_measurement",
    "orientation": "topology_occurrence",
    "ownership_licence": "source_import_normalization",
    "pcurve_mapping": "coedge",
    "periodicity": "periodicity_singularity",
    "recognition_evidence": "semantic_evidence",
    "report_determinism": "consumer_evidence",
    "semantic_candidate": "semantic_evidence",
    "singularity": "periodicity_singularity",
    "source_provenance": "source_import_normalization",
    "source_transform": "transform",
    "step_import": "source_import_normalization",
    "support_assignment": "consumer_evidence",
    "surface_geometry": "geometry",
    "surface_representation": "geometry",
    "topology_counts": "topology_occurrence",
    "topology_identity": "topology_occurrence",
    "topology_incidence": "topology_occurrence",
    "topology_occurrences": "topology_occurrence",
    "trim_domain": "trim",
    "validation_finding": "pathology_measurement",
    "vertex_parameters": "topology_occurrence",
    "viewer_state": "consumer_evidence",
}

IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value: dict[str, Any]) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="ascii",
        newline="\n",
    )


def wrapper_code(concrete_type: str) -> str:
    mapping = {
        "Geom_TrimmedCurve": "trimmed",
        "Geom_OffsetCurve": "offset",
        "Geom_RectangularTrimmedSurface": "rectangular_trimmed",
        "Geom_OffsetSurface": "offset",
    }
    return mapping.get(concrete_type, "kernel_specific")


def parameter_domain(
    axis: str, closed: bool, periodic: bool, period: float | None = None
) -> dict[str, Any]:
    return {
        "axis": axis,
        "closed": closed,
        "lower": None,
        "period": period if periodic else None,
        "periodic": periodic,
        "seam_parameters": [],
        "unit": "radians" if periodic else "unitless",
        "upper": None,
    }


def geometry_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    periodicity = {item["label"]: item for item in expected["periodicity"]}
    variants = [
        ("Curve", item) for item in expected["curve_variants"]
    ] + [("Surface", item) for item in expected["surface_variants"]]
    for taxonomy, item in variants:
        conditions = item["condition_codes"]
        wrappers = [
            {
                "basis_observation_id": None,
                "code": wrapper_code(concrete),
                "concrete_type": concrete,
                "ordinal": index,
                "parameters": [],
            }
            for index, concrete in enumerate(item["wrapper_stack"])
        ]
        if taxonomy == "Curve":
            closed = any("closed" in code for code in conditions)
            periodic = any("periodic" in code for code in conditions)
            domains = [
                parameter_domain(
                    "curve", closed, periodic, 2.0 * math.pi if periodic else None
                )
            ]
        else:
            state = periodicity.get(item["label"], {})
            domains = []
            for axis in ("u", "v"):
                periodic = bool(state.get(f"{axis}_periodic", False))
                domains.append(
                    parameter_domain(
                        axis,
                        bool(state.get(f"{axis}_closed", False)),
                        periodic,
                        2.0 * math.pi if periodic else None,
                    )
                )
        result.append(
            {
                "concrete_type": item["concrete_type"],
                "condition_codes": conditions,
                "exact_evaluator_kind": f"occt.exact_{taxonomy.lower()}",
                "family_code": item["family_code"],
                "id": f"observation.geometry.{item['label']}",
                "kind": "geometry",
                "parameter_domains": domains,
                "parameters": [],
                "phase": "imported",
                "subject_id": item["label"],
                "taxonomy": taxonomy,
                "wrappers": wrappers,
            }
        )
    return result


def topology_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    plural_to_kind = {
        "compounds": "compound",
        "compsolids": "compsolid",
        "solids": "solid",
        "shells": "shell",
        "faces": "face",
        "wires": "wire",
        "edges": "edge",
        "vertices": "vertex",
    }
    orientation = expected["orientation_states"][0]
    for plural, kind in plural_to_kind.items():
        count = expected["unique_topology"][plural]
        if count == 0:
            continue
        subject = f"topology.{plural}"
        result.append(
            {
                "child_ids": [],
                "condition_codes": ["oracle.aggregate_unique_inventory"],
                "has_exact_representation": True,
                "id": f"observation.topology.{kind}",
                "instance_id": None,
                "kind": "topology_occurrence",
                "orientation": orientation,
                "parent_id": None,
                "phase": "imported",
                "relations": [
                    {
                        "code": "oracle.unique_count",
                        "parameter": {"unit": "unitless", "value": count},
                        "semantic_ordinal": None,
                        "target_id": subject,
                    }
                ],
                "source_matrix_row_major": IDENTITY,
                "subject_id": subject,
                "tolerance_metres": 0,
                "topology_kind": kind,
                "underlying_id": subject,
                "world_matrix_row_major": IDENTITY,
            }
        )
    return result


def trim_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    loops = expected["boundary_loops"]
    result: list[dict[str, Any]] = []
    for variant in expected["surface_variants"]:
        converted = []
        for loop in loops:
            bounds = loop["uv_bounds"]
            area = (bounds["u_max"] - bounds["u_min"]) * (
                bounds["v_max"] - bounds["v_min"]
            )
            if loop["orientation"] == "REVERSED":
                area = -area
            coedges = [
                f"oracle.coedge.{loop['id']}.c{index + 1}"
                for index in range(loop["edge_occurrence_count"])
            ]
            converted.append(
                {
                    "closed": loop["closed"],
                    "coedge_ids": coedges,
                    "loop_id": loop["id"],
                    "max_gap_metres": 0,
                    "nesting_depth": 0 if loop["role"] == "outer" else 1,
                    "nesting_parent_loop_id": None,
                    "orientation": loop["orientation"],
                    "role": loop["role"],
                    "seam_crossing_count": 0,
                    "seam_unwrap_offsets": [],
                    "self_intersection_count": 0,
                    "signed_uv_area": area,
                    "singularity_ids": [],
                    "uv_bounds": bounds,
                    "wire_id": f"oracle.wire.{loop['id']}",
                }
            )
        result.append(
            {
                "condition_codes": variant["condition_codes"],
                "disconnected_region_count": 1,
                "face_id": variant["label"],
                "id": f"observation.trim.{variant['label']}",
                "kind": "trim",
                "loops": converted,
                "phase": "imported",
                "trim_domain_class": variant["trim_domain_class"],
            }
        )
    return result


def periodicity_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    result = []
    for item in expected["periodicity"]:
        axes = []
        for axis in ("u", "v"):
            periodic = item[f"{axis}_periodic"]
            axes.append(
                {
                    "axis": axis,
                    "closed": item[f"{axis}_closed"],
                    "period": 2.0 * math.pi if periodic else None,
                    "periodic": periodic,
                    "principal_lower": 0 if periodic else None,
                    "principal_upper": 2.0 * math.pi if periodic else None,
                    "seam_parameters": [0] if periodic else [],
                    "unit": "radians" if periodic else "unitless",
                }
            )
        singularities = []
        for index, code in enumerate(expected["singularity_codes"]):
            kind = "apex" if "apex" in code else "pole" if "pole" in code else "kernel_specific"
            singularities.append(
                {
                    "code": code,
                    "incident_subject_ids": [item["label"]],
                    "kind": kind,
                    "point_metres": None,
                    "singularity_id": f"singularity.{index + 1}",
                    "topology_disposition": "unresolved",
                    "u_parameter": None,
                    "unit_normal_defined": False,
                    "v_parameter": None,
                }
            )
        result.append(
            {
                "axes": axes,
                "id": f"observation.periodicity.{item['label']}",
                "kind": "periodicity_singularity",
                "phase": "imported",
                "singularities": singularities,
                "subject_id": item["label"],
            }
        )
    return result


def semantic_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        {
            "candidate_code": item["code"],
            "confidence": item["confidence"],
            "continuity_observation_ids": [],
            "evidence": item["evidence"],
            "evidence_codes": ["evidence.independent_oracle"],
            "fitted_parameters": [],
            "id": f"observation.semantic.{item['code']}",
            "kind": "semantic_evidence",
            "max_residual_metres": None,
            "phase": "imported",
            "residual_observation_ids": [],
            "subject_ids": [item["code"]],
        }
        for item in expected["semantic_candidates"]
    ]


def pathology_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    result = []
    for index, item in enumerate(expected["validation_findings"]):
        observed_count = 0 if item["expectation"] == "absent" else 1
        result.append(
            {
                "evidence": item["evidence"],
                "expectation": item["expectation"],
                "expected_failure_code": None,
                "id": f"observation.pathology.p{index + 1}",
                "kind": "pathology_measurement",
                "measurements": [
                    {
                        "code": f"{item['code']}.count",
                        "comparison": "exact",
                        "sample_count": None,
                        "tolerance": None,
                        "value": {
                            "unit": "count",
                            "value": observed_count,
                            "value_type": "integer",
                        },
                    }
                ],
                "pathology_code": item["code"],
                "phase": "imported",
                "subject_ids": [f"oracle.finding.p{index + 1}"],
            }
        )
    return result


def normalization_observation() -> dict[str, Any]:
    return {
        "accepted_normalization_codes": [],
        "changes": [],
        "equivalence": "exact",
        "geometry_equivalent": True,
        "id": "observation.normalization.model",
        "imported_subject_ids": ["imported.model"],
        "kind": "source_import_normalization",
        "outcome": "preserved",
        "source_subject_ids": ["source.model"],
        "topology_equivalent": True,
    }


def determinant(matrix: list[float]) -> float:
    return (
        matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9])
        - matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8])
        + matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8])
    )


def transform_observations(expectation: dict[str, Any]) -> list[dict[str, Any]]:
    instances = expectation["instances"] or [
        {
            "label": "instance.synthetic_identity",
            "local_matrix_row_major": IDENTITY,
            "matrix_row_major": IDENTITY,
            "parent_label": None,
            "target_definition": "definition.synthetic_identity",
            "transform_kind": "identity",
        }
    ]
    result = []
    for item in instances:
        local = item["local_matrix_row_major"]
        det = determinant(local)
        result.append(
            {
                "baked_geometry_matrix_row_major": None,
                "disposition": "retained_as_location",
                "handedness": "right_handed" if det > 0 else "left_handed" if det < 0 else "singular",
                "id": f"observation.transform.{item['label']}",
                "imported_location_matrix_row_major": local,
                "instance_id": item["label"],
                "kind": "transform",
                "linear_determinant": det,
                "parent_instance_id": item["parent_label"],
                "source_affine_matrix_row_major": local,
                "source_unit_scale_to_metres": expectation["units"]["scale_to_metres"],
                "target_definition_id": item["target_definition"],
                "transform_kind": item["transform_kind"],
                "world_matrix_row_major": item["matrix_row_major"],
            }
        )
    return result


def consumer_observations(expected: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        {
            "artifact": None,
            "assertion_codes": [item["selection_code"]],
            "consumer_code": "oracle.support_assignment",
            "consumer_kind": "inventory",
            "evidence": (
                f"Independent oracle assigns {item['subject_count']} {item['subject_kind']} "
                f"subjects to {item['state']} using {item['selection_code']}."
            ),
            "expected_outcome": "present",
            "id": f"observation.consumer.{item['label']}",
            "kind": "consumer_evidence",
            "measurements": [],
            "representation_kind": "inventory_report",
            "subject_ids": item["subject_ids"],
        }
        for item in expected["support"]
    ]


def observation_matches_refs(observation: dict[str, Any], reference_ids: set[str]) -> bool:
    candidates = {
        observation.get("subject_id"),
        observation.get("face_id"),
        observation.get("candidate_code"),
        observation.get("instance_id"),
        observation.get("pathology_code"),
    }
    candidates.update(observation.get("subject_ids", []))
    return bool(reference_ids & {value for value in candidates if value is not None})


def populate(path: Path, profiles: dict[str, Any]) -> None:
    document = load(path)
    expected = document["expected"]
    observations = (
        geometry_observations(expected)
        + topology_observations(expected)
        + trim_observations(expected)
        + periodicity_observations(expected)
        + semantic_observations(expected)
        + pathology_observations(expected)
        + [normalization_observation()]
        + transform_observations(document)
        + consumer_observations(expected)
    )
    observations.sort(key=lambda item: item["id"])
    document["observations"] = observations
    by_kind: dict[str, list[dict[str, Any]]] = {}
    for observation in observations:
        by_kind.setdefault(observation["kind"], []).append(observation)

    for claim in expected["claims"]:
        profile = profiles[claim["obligation_id"]]
        required_kinds = {
            PROFILE_TO_KIND[item] for item in profile["observation_kinds"]
        }
        reference_ids = {item["id"] for item in claim["subject_refs"]}
        for kind in sorted(required_kinds):
            available = by_kind.get(kind, [])
            matching = [
                item for item in available if observation_matches_refs(item, reference_ids)
            ]
            selected = matching or available
            if not selected:
                raise ValueError(
                    f"{document['fixture_id']} {claim['id']} has no {kind} observation"
                )
            claim["subject_refs"].extend(
                {"id": item["id"], "kind": "observation"} for item in selected
            )
        claim["subject_refs"] = sorted(
            { (item["kind"], item["id"]): item for item in claim["subject_refs"] }.values(),
            key=lambda item: (item["kind"], item["id"]),
        )
    write(path, document)


def main() -> int:
    profiles = load(PROFILES_PATH)["obligation_profiles"]
    manifest = load(MANIFEST_PATH)
    paths = {
        (ROOT / "fixtures" / fixture["expectation_path"]).resolve()
        for fixture in manifest["fixtures"]
        if fixture["source"]["kind"] == "occt_procedural"
    }
    for path in sorted(paths):
        populate(path, profiles)
    print(f"populated independent observations in {len(paths)} expectations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
