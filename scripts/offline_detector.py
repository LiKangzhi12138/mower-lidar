"""Configurable offline *candidate* extraction; no vehicle-frame or ground truth claims."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import platform
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from extract_obstacle_candidates import (
    candidate_color, cluster_dbscan, describe_candidate,
    extract_plane_candidate, sha256_file, write_cloud, write_preview,
)
from npz_to_ply import read_cloud_data
from patchwork_ground import split_ground_indices
from prepare_cloud import filter_point_rows


ROOT = Path(__file__).resolve().parent.parent


def _unit_vector(value, name):
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite 3-vector")
    norm = np.linalg.norm(vector)
    if norm == 0:
        raise ValueError(f"{name} must be nonzero")
    return vector / norm


def analysis_coordinates(sensor_xyz, settings):
    """Use raw sensor coordinates, or an explicitly parameterized trial mount transform."""
    mode = settings.get("mode", "raw_sensor")
    if mode == "raw_sensor":
        if settings.get("mount_height") is not None or settings.get("mount_pitch_degrees") is not None:
            raise ValueError("mount height/pitch require coordinates.mode=experimental_mount")
        return sensor_xyz.astype(np.float64, copy=True), settings.get("sensor_frame", "sensor_raw")
    if mode != "experimental_mount":
        raise ValueError("coordinates.mode must be raw_sensor or experimental_mount")
    required = ("mount_height", "mount_pitch_degrees", "pitch_axis_sensor", "vertical_axis_analysis", "analysis_frame")
    if any(settings.get(key) is None for key in required):
        raise ValueError(f"experimental_mount needs {required}")
    height = float(settings["mount_height"])
    angle = np.deg2rad(float(settings["mount_pitch_degrees"]))
    if not np.isfinite(height) or not np.isfinite(angle):
        raise ValueError("mount height/pitch must be finite")
    axis = _unit_vector(settings["pitch_axis_sensor"], "pitch_axis_sensor")
    vertical = _unit_vector(settings["vertical_axis_analysis"], "vertical_axis_analysis")
    cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rotation = np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)
    transformed = sensor_xyz.astype(np.float64) @ rotation.T + height * vertical
    return transformed, settings["analysis_frame"]


def euclidean_clusters(xyz, radius, min_points, max_points=None):
    """Connected components of a radius graph; -1 labels rejected components."""
    if not np.isfinite(radius) or radius <= 0 or min_points < 1:
        raise ValueError("Invalid Euclidean clustering parameters")
    if max_points is not None and max_points < min_points:
        raise ValueError("max_points must be >= min_points")
    labels = np.full(len(xyz), -1, dtype=np.int32)
    visited = np.zeros(len(xyz), dtype=bool)
    tree = cKDTree(xyz) if len(xyz) else None
    cluster_id = 0
    for start in range(len(xyz)):
        if visited[start]:
            continue
        visited[start] = True
        component = [start]
        cursor = 0
        while cursor < len(component):
            point = component[cursor]
            cursor += 1
            for neighbor in tree.query_ball_point(xyz[point], radius):
                if not visited[neighbor]:
                    visited[neighbor] = True
                    component.append(neighbor)
        if len(component) >= min_points and (max_points is None or len(component) <= max_points):
            labels[component] = cluster_id
            cluster_id += 1
    return labels


def filter_height_indices(xyz, vertical_axis_analysis, reference_level, min_clearance):
    """Return rows below an explicitly configured trial clearance boundary."""
    vertical = _unit_vector(vertical_axis_analysis, "vertical_axis_analysis")
    reference = float(reference_level)
    clearance = float(min_clearance)
    if not np.isfinite(reference) or not np.isfinite(clearance) or clearance < 0:
        raise ValueError("Invalid height baseline reference/clearance")
    return np.flatnonzero((np.asarray(xyz) @ vertical - reference) < clearance)


def normal_constrained_plane(xyz, params):
    """Find a horizontal trial plane while retaining every rejected dominant plane."""
    vertical = _unit_vector(params["vertical_axis_analysis"], "vertical_axis_analysis")
    max_angle = float(params["max_normal_angle_degrees"])
    max_intercept = float(params["max_abs_ground_intercept"])
    max_trials = int(params["max_plane_trials"])
    if (not np.isfinite(max_angle) or not np.isfinite(max_intercept) or
            not 0 <= max_angle < 90 or max_intercept < 0 or max_trials < 1):
        raise ValueError("Invalid ground normal/intercept search parameters")
    remaining = np.arange(len(xyz), dtype=np.int64)
    rejected = []
    for trial in range(max_trials):
        if len(remaining) < params["ransac_n"]:
            break
        try:
            model, local = extract_plane_candidate(
                xyz[remaining], params["distance_threshold"], params["ransac_n"],
                params["num_iterations"], params["probability"],
                params["random_seed"] + trial,
            )
        except RuntimeError:
            break
        if not len(local):
            break
        normal = model[:3]
        norm = np.linalg.norm(normal)
        if not np.isfinite(model).all() or norm < 1e-12:
            break
        alignment = abs(float(np.dot(normal, vertical))) / norm
        angle = float(np.rad2deg(np.arccos(np.clip(alignment, -1.0, 1.0))))
        intercept = abs(float(model[3])) / abs(float(np.dot(normal, vertical))) if alignment > 1e-12 else float("inf")
        record = {"plane_model": model.tolist(), "point_count": int(len(local)),
                  "normal_angle_degrees": angle, "abs_vertical_intercept": intercept}
        if angle <= max_angle and intercept <= max_intercept:
            return model, remaining[local], rejected, record
        rejected.append(record)
        remaining = np.delete(remaining, local)
    return None, np.empty(0, dtype=np.int64), rejected, None


def _write_ply(path, xyz, colors):
    if len(xyz):
        write_cloud(path, xyz, colors)
    else:
        # Open3D rejects zero-vertex PLY on read; absence is recorded by counts.
        path.unlink(missing_ok=True)


def _solid(n, color):
    return np.tile(np.asarray(color, dtype=np.float64), (n, 1))


def _jsonable(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def _software_versions():
    versions = {"python": platform.python_version()}
    for package in ("numpy", "open3d", "scipy", "pypatchworkpp"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "local package metadata unavailable"
    return versions


def run(input_path: Path, config_path: Path, output_dir: Path, run_name: str):
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    if not config_path.is_file():
        raise FileNotFoundError(config_path)
    if not run_name or Path(run_name).name != run_name or run_name in (".", ".."):
        raise ValueError("run_name must be one safe directory name")
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    if cfg.get("schema_version") != 1:
        raise ValueError("Expected schema_version=1")
    for section in ("preprocess", "coordinates", "ground", "clustering"):
        if not isinstance(cfg.get(section), dict):
            raise ValueError(f"Missing {section} configuration")
    if cfg["coordinates"].get("unit_confirmed", False) and not cfg["coordinates"].get("coordinate_unit"):
        raise ValueError("Confirmed units require coordinate_unit")

    started = time.perf_counter()
    t = started
    sensor_all, intensity_all, fields = read_cloud_data(input_path, verbose=False)
    sensor_all = np.asarray(sensor_all)
    keep, filter_stats = filter_point_rows(sensor_all, intensity_all, cfg["preprocess"].get("zero_policy", "exclude"))
    sensor = sensor_all[keep].astype(np.float64, copy=False)
    intensity = None if intensity_all is None else np.asarray(intensity_all).reshape(-1)[keep]
    source_indices = np.flatnonzero(keep).astype(np.int64)
    timings = {"read_preprocess_seconds": time.perf_counter() - t}

    t = time.perf_counter()
    xyz, frame = analysis_coordinates(sensor, cfg["coordinates"])
    timings["coordinate_transform_seconds"] = time.perf_counter() - t

    ground_cfg = cfg["ground"]
    ground_method = ground_cfg.get("method", "none")
    all_idx = np.arange(len(xyz), dtype=np.int64)
    removed = np.empty(0, dtype=np.int64)
    plane = np.empty(0, dtype=np.int64)
    plane_model = None
    effective_ground = None
    unclassified = np.empty(0, dtype=np.int64)
    rejected_planes = []
    selected_plane_diagnostics = None
    ground_note = "No ground removal; all cleaned points remain candidates."
    t = time.perf_counter()
    if ground_method == "none":
        pass
    elif ground_method == "ransac":
        params = ground_cfg["params"]
        if (params["distance_threshold"] <= 0 or params["ransac_n"] < 3 or
                params["num_iterations"] < 1 or not 0 < params["probability"] <= 1 or
                params["random_seed"] < 0):
            raise ValueError("Invalid RANSAC parameters")
        if len(xyz) >= params["ransac_n"]:
            try:
                plane_model, plane = extract_plane_candidate(
                    xyz, params["distance_threshold"], params["ransac_n"],
                    params["num_iterations"], params["probability"], params["random_seed"],
                )
                if (not np.isfinite(plane_model).all() or
                        np.linalg.norm(plane_model[:3]) < 1e-12 or
                        len(plane) < params["ransac_n"]):
                    plane_model, plane = None, np.empty(0, dtype=np.int64)
            except RuntimeError:
                plane_model, plane = None, np.empty(0, dtype=np.int64)
        policy = ground_cfg.get("plane_policy", "retain")
        if policy == "exclude_experimental":
            removed = plane
        elif policy != "retain":
            raise ValueError("plane_policy must be retain or exclude_experimental")
        ground_note = "RANSAC plane is geometric only; no ground identity verified."
    elif ground_method == "patchwork":
        # Patchwork++ expects a level, sensor-centred cloud: its sensor_height
        # represents the offset from that origin to the ground. The analysis
        # frame above includes the trial origin-to-ground translation.
        patchwork_xyz = xyz
        if cfg["coordinates"].get("mode") == "experimental_mount":
            mount_height = float(cfg["coordinates"]["mount_height"])
            vertical = _unit_vector(cfg["coordinates"]["vertical_axis_analysis"], "vertical_axis_analysis")
            if not np.isclose(mount_height, ground_cfg["params"]["sensor_height"]):
                raise ValueError("Patchwork++ sensor_height must match trial mount_height")
            patchwork_xyz = xyz - mount_height * vertical
        removed, nonground, unclassified, effective_ground = split_ground_indices(
            patchwork_xyz, intensity, ground_cfg["params"]
        )
        ground_note = (
            "Patchwork++ ground labels are experimental; unclassified points remain "
            "in the candidate cloud."
        )
    elif ground_method == "ransac_normal":
        params = ground_cfg["params"]
        if (params["distance_threshold"] <= 0 or params["ransac_n"] < 3 or
                params["num_iterations"] < 1 or not 0 < params["probability"] <= 1 or
                params["random_seed"] < 0):
            raise ValueError("Invalid RANSAC parameters")
        plane_model, plane, rejected_planes, selected_plane_diagnostics = normal_constrained_plane(xyz, params)
        removed = plane
        ground_note = (
            "Experimental normal- and intercept-constrained RANSAC ground candidate; "
            "rejected dominant planes remain in the nonground candidate cloud."
        )
    elif ground_method == "height":
        params = ground_cfg["params"]
        # This is an explicit trial axis/reference, never inferred from sensor Z.
        removed = filter_height_indices(
            xyz, params["vertical_axis_analysis"],
            params["reference_level"], params["min_clearance"],
        )
        ground_note = "Height baseline uses an explicitly assumed trial vertical axis/reference."
    else:
        raise ValueError(f"Unknown ground method: {ground_method}")
    candidate_rows = np.setdiff1d(all_idx, removed)
    timings["ground_or_plane_seconds"] = time.perf_counter() - t

    clustering_cfg = cfg["clustering"]
    cluster_method = clustering_cfg.get("method", "none")
    trial = xyz[candidate_rows]
    t = time.perf_counter()
    if cluster_method == "none":
        labels = np.full(len(trial), -1, dtype=np.int32)
    elif cluster_method == "dbscan":
        p = clustering_cfg["params"]
        if p["eps"] <= 0 or p["min_points"] < 1:
            raise ValueError("Invalid DBSCAN parameters")
        labels = cluster_dbscan(trial, p["eps"], p["min_points"])
    elif cluster_method == "euclidean":
        p = clustering_cfg["params"]
        labels = euclidean_clusters(trial, p["radius"], p["min_points"], p.get("max_points"))
    else:
        raise ValueError(f"Unknown clustering method: {cluster_method}")
    timings["clustering_seconds"] = time.perf_counter() - t

    t = time.perf_counter()
    target = output_dir / run_name
    target.mkdir(parents=True, exist_ok=True)
    candidates = []
    colors = _solid(len(trial), [0.45, 0.45, 0.45])
    for cluster_id in np.unique(labels[labels >= 0]):
        local = np.flatnonzero(labels == cluster_id)
        rows = candidate_rows[local]
        summary = describe_candidate(xyz[rows], sensor[rows], source_indices[rows])
        filename = f"candidate_{int(cluster_id):03d}.ply"
        color = candidate_color(int(cluster_id))
        colors[local] = color
        _write_ply(target / filename, xyz[rows], _solid(len(rows), color))
        candidates.append({
            "id": int(cluster_id), "classification": "experimental_obstacle_candidate",
            "point_count": len(rows), "point_cloud": filename,
            "input_indices": source_indices[rows].tolist(), **summary,
        })
    _write_ply(target / "candidate_region.ply", trial, _solid(len(trial), [0.8, 0.8, 0.8]))
    _write_ply(target / "clusters_colored.ply", trial, colors)
    _write_ply(target / "plane_candidate.ply", xyz[plane], _solid(len(plane), [0.1, 0.3, 0.95]))
    _write_ply(target / "removed_experimental.ply", xyz[removed], _solid(len(removed), [0.1, 0.85, 0.2]))
    np.savez_compressed(
        target / "indices_and_labels.npz",
        clean_source_indices=source_indices,
        candidate_source_indices=source_indices[candidate_rows],
        removed_source_indices=source_indices[removed],
        unclassified_source_indices=source_indices[unclassified],
        plane_source_indices=source_indices[plane],
        candidate_labels=labels,
    )
    if cfg.get("visualization", {}).get("preview_png", False) and len(trial):
        write_preview(target / "clusters_preview.png", trial, colors, run_name,
                      "colored: candidates, gray: unclustered candidate points")
    timings["output_seconds"] = time.perf_counter() - t
    timings["total_seconds"] = time.perf_counter() - started

    result = {
        "result_type": "experimental_obstacle_candidates",
        "input_file": str(input_path.resolve()), "input_sha256": sha256_file(input_path),
        "source_fields": fields, "config_file": str(config_path.resolve()),
        "software_versions": _software_versions(),
        "parameters": cfg, "effective_patchwork_parameters": effective_ground,
        "analysis_frame": frame, "sensor_frame": cfg["coordinates"].get("sensor_frame", "sensor_raw"),
        "coordinate_unit": cfg["coordinates"].get("coordinate_unit"),
        "coordinate_unit_confirmed": bool(cfg["coordinates"].get("unit_confirmed", False)),
        "ground_note": ground_note, "ground_method": ground_method,
        "plane_model_ax_by_cz_d": None if plane_model is None else plane_model.tolist(),
        "plane_candidate_point_count": len(plane), "removed_point_count": len(removed),
        "unclassified_point_count": len(unclassified),
        "rejected_plane_candidates": rejected_planes,
        "selected_plane_diagnostics": selected_plane_diagnostics,
        "candidate_region_point_count": len(candidate_rows),
        "noise_point_count": int((labels == -1).sum()),
        "candidate_count": len(candidates), "candidates": candidates,
        "preprocess_stats": filter_stats, "timings": timings,
        "distance_reference": "raw_sensor_origin",
        "distance_unit": cfg["coordinates"].get("coordinate_unit") if cfg["coordinates"].get("unit_confirmed", False) else "unconfirmed_coordinate_values",
    }
    (target / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, default=_jsonable), encoding="utf-8")
    return result, target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "r3_offline_detector")
    parser.add_argument("--run-name", required=True)
    args = parser.parse_args()
    result, target = run(args.input, args.config, args.output_dir, args.run_name)
    print(json.dumps({"output": str(target), "remaining": result["preprocess_stats"]["remaining_point_count"],
                      "candidate_region": result["candidate_region_point_count"],
                      "candidate_count": result["candidate_count"],
                      "total_seconds": result["timings"]["total_seconds"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
