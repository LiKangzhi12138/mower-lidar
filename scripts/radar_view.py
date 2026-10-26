"""Experimental XY bearing and static radar plot for existing R3 candidates.

This module never classifies points. It reads an offline_detector result and
its source-index map, then reuses the recorded coordinate transform.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import numpy as np
import open3d as o3d

from extract_obstacle_candidates import candidate_color, sha256_file
from npz_to_ply import read_cloud_data
from offline_detector import analysis_coordinates
from prepare_cloud import filter_point_rows


ANGLE_REFERENCE = "+X=0deg; +Y=+90deg; counterclockwise in current XY frame"


def xy_polar(center_xyz, cluster_xyz, frame):
    """2-D range and azimuth in exactly the frame of center/cluster XYZ."""
    center = np.asarray(center_xyz, dtype=np.float64)
    points = np.asarray(cluster_xyz, dtype=np.float64)
    if center.shape != (3,) or points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError("Need a 3-D center and nonempty Nx3 candidate points")
    if not np.isfinite(center).all() or not np.isfinite(points).all():
        raise ValueError("Non-finite candidate coordinates")
    distance = float(np.hypot(center[0], center[1]))
    defined = bool(distance > 0)
    return {
        "range_xy": distance,
        "nearest_range_xy": float(np.min(np.hypot(points[:, 0], points[:, 1]))),
        "azimuth_deg": float(np.degrees(np.arctan2(center[1], center[0]))) if defined else 0.0,
        "azimuth_defined": defined,
        "coordinate_frame": frame,
        "angle_reference": ANGLE_REFERENCE,
    }


def display_xy(xy):
    """Map analysis +X to screen up and analysis +Y to screen left."""
    xy = np.asarray(xy, dtype=np.float64)
    if xy.shape[-1] != 2:
        raise ValueError("Expected (...,2) XY coordinates")
    return np.stack((-xy[..., 1], xy[..., 0]), axis=-1)


def choose_rings(max_cluster_range, max_radius=None, ring_step=None):
    if max_radius is not None:
        radius = float(max_radius)
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError("max_radius must be positive and finite")
    else:
        target = max(1.0, float(max_cluster_range) * 1.1)
        exponent = 10 ** math.floor(math.log10(target / 4))
        step = next(value * exponent for value in (1, 2, 2.5, 5, 10) if value * exponent >= target / 4)
        radius = math.ceil(target / step) * step
    if ring_step is None:
        target_step = radius / 4
        exponent = 10 ** math.floor(math.log10(target_step))
        step = next(value * exponent for value in (1, 2, 2.5, 5, 10) if value * exponent >= target_step)
    else:
        step = float(ring_step)
        if not np.isfinite(step) or step <= 0:
            raise ValueError("ring_step must be positive and finite")
    rings = np.arange(step, radius + step * 1e-8, step, dtype=np.float64)
    return radius, rings


def draw_radar(path, candidates, analysis_xyz, candidate_rows, labels, *,
               sample_name, frame, unit_note, calibration_note, max_radius=None,
               ring_step=None, show_noise=True, show_boxes=False):
    clustered = labels >= 0
    cluster_points = analysis_xyz[candidate_rows[clustered]]
    maximum = float(np.max(np.hypot(cluster_points[:, 0], cluster_points[:, 1]))) if len(cluster_points) else 0.0
    radius, rings = choose_rings(maximum, max_radius, ring_step)
    fig = plt.figure(figsize=(13.5, 10), facecolor="#07131e")
    ax = fig.add_axes([0.045, 0.09, 0.65, 0.82], facecolor="#07131e")
    info = fig.add_axes([0.72, 0.11, 0.265, 0.78], facecolor="#0b1c2b")
    info.set_axis_off()
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-radius, radius); ax.set_ylim(-radius, radius)
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color("#36516a")
    for value in rings:
        ax.add_patch(Circle((0, 0), float(value), fill=False,
                            edgecolor="#2d6374", linewidth=1.0, linestyle="--", alpha=0.7))
        ax.text(-0.12 * value, 0.96 * value, f"{value:g}", color="#78afb9",
                fontsize=8, ha="right", va="center")
    ax.axhline(0, color="#558ca2", linewidth=1.1, alpha=0.85)
    ax.axvline(0, color="#558ca2", linewidth=1.1, alpha=0.85)
    for angle in (-135, -90, -45, 0, 45, 90, 135, 180):
        theta = math.radians(angle)
        x, y = display_xy([math.cos(theta), math.sin(theta)])
        if angle not in (0, 90, -90, 180):
            ax.plot([0, radius * x], [0, radius * y], color="#234759", linewidth=0.7, alpha=0.6)
            ax.text(0.9 * radius * x, 0.9 * radius * y, f"{angle:+d}°",
                    color="#7c9eaf", fontsize=8, ha="center", va="center")
    for x, y, title in ((0, 0.96, "+X / 0°"), (-0.96, 0, "+Y / +90°"),
                        (0.96, 0, "-Y / -90°"), (0, -0.96, "-X / ±180°")):
        ax.text(radius * x, radius * y, title, color="#b2dce5", fontsize=9,
                ha="center", va="center", weight="bold")
    ax.scatter([0], [0], s=72, marker="+", linewidths=2.5, color="#f7e375", zorder=10)
    noise_visible = 0
    noise_total = int(np.count_nonzero(labels < 0))
    if show_noise and noise_total:
        noise = analysis_xyz[candidate_rows[labels < 0], :2]
        inside = np.hypot(noise[:, 0], noise[:, 1]) <= radius
        shown = display_xy(noise[inside])
        noise_visible = len(shown)
        ax.scatter(shown[:, 0], shown[:, 1], s=1.0, c="#72818d", alpha=0.26, linewidths=0)
    for item in candidates:
        cid = item["id"]
        color = candidate_color(cid)
        points = analysis_xyz[candidate_rows[labels == cid], :2]
        inside = np.hypot(points[:, 0], points[:, 1]) <= radius
        shown = display_xy(points[inside])
        ax.scatter(shown[:, 0], shown[:, 1], s=5.0, color=color, alpha=0.75, linewidths=0)
        if show_boxes:
            lower = item["bbox_min_xyz"][:2]
            upper = item["bbox_max_xyz"][:2]
            corners = np.array([[lower[0], lower[1]], [upper[0], lower[1]],
                                [upper[0], upper[1]], [lower[0], upper[1]]])
            ax.add_patch(Polygon(display_xy(corners), closed=True, fill=False,
                                 edgecolor=color, linewidth=1.0, alpha=0.7))
        cx, cy = display_xy(item["center_xyz"][:2])
        ax.scatter([cx], [cy], s=65, facecolor="#07131e", edgecolor=color,
                   linewidths=1.7, zorder=11)
        ax.annotate(f"#{cid}", (cx, cy), xytext=(5, 5), textcoords="offset points",
                    fontsize=8, color="white", weight="bold", zorder=12,
                    bbox={"facecolor": "#07131e", "edgecolor": "none", "alpha": 0.65, "pad": 1})
    info.text(0.03, 0.98, "OBSTACLE CANDIDATES", color="#ddf4f3", size=13,
              weight="bold", va="top", transform=info.transAxes)
    info.text(0.03, 0.94, "ID   center X,Y     range  azimuth", color="#8db5c3",
              size=8.5, va="top", transform=info.transAxes)
    per_row = min(0.037, 0.87 / max(1, len(candidates)))
    for index, item in enumerate(candidates):
        y = 0.91 - index * per_row
        color = candidate_color(item["id"])
        info.scatter([0.042], [y], s=55, color=color, transform=info.transAxes)
        center = item["center_xyz"]
        info.text(0.085, y, f"{item['id']:02d}   {center[0]:+5.1f},{center[1]:+5.1f}   "
                  f"{item['range_xy']:5.1f}   {item['azimuth_deg']:+6.1f}°",
                  color="#d8e7e9", size=8.5, va="center", family="monospace",
                  transform=info.transAxes)
    fig.text(0.05, 0.955, f"MID-360  |  {sample_name}  |  EXPERIMENTAL OBSTACLE VIEW",
             color="#ecf9f7", fontsize=18, weight="bold")
    fig.text(0.05, 0.925, f"Frame: {frame}  |  +X up, +Y left  |  radial unit: {unit_note}",
             color="#b8d1d7", fontsize=10)
    fig.text(0.05, 0.032, f"Calibration: {calibration_note}  •  Rings: XY distance from analysis origin",
             color="#92adbb", fontsize=9)
    if show_noise:
        fig.text(0.72, 0.07, f"Gray DBSCAN noise: {noise_visible}/{noise_total} shown inside {radius:g}",
                 color="#94a7ae", fontsize=8)
    fig.savefig(path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    return {
        "plot_radius": radius, "ring_radii": rings.tolist(),
        "display_mapping": "screen_horizontal=-analysis_Y; screen_vertical=analysis_X",
        "equal_xy_scale": True, "clustered_points_outside_plot": int(np.count_nonzero(
            np.hypot(cluster_points[:, 0], cluster_points[:, 1]) > radius)),
        "unclustered_points_total": noise_total, "unclustered_points_shown": noise_visible,
        "show_boxes": show_boxes, "show_noise": show_noise,
    }


def export_result(result_path, output_dir, *, max_radius=None, ring_step=None,
                  show_noise=True, show_boxes=False):
    result_path = Path(result_path)
    output_dir = Path(output_dir)
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result["ground_method"] != "ransac_normal" or result["parameters"]["clustering"]["method"] != "dbscan":
        raise ValueError("R3.4 requires existing constrained-RANSAC + DBSCAN result")
    source = Path(result["input_file"])
    if sha256_file(source) != result["input_sha256"]:
        raise ValueError("Input NPZ hash differs from recorded R3 result")
    raw, intensity, fields = read_cloud_data(source, verbose=False)
    keep, stats = filter_point_rows(raw, intensity, result["parameters"]["preprocess"]["zero_policy"])
    source_indices = np.flatnonzero(keep)
    analysis_xyz, frame = analysis_coordinates(np.asarray(raw)[keep], result["parameters"]["coordinates"])
    if frame != result["analysis_frame"] or fields != result["source_fields"]:
        raise ValueError("Frame or source fields differ from recorded run")
    with np.load(result_path.parent / "indices_and_labels.npz") as mapping:
        clean_source = mapping["clean_source_indices"].copy()
        candidate_source = mapping["candidate_source_indices"].copy()
        labels = mapping["candidate_labels"].copy()
    if not np.array_equal(clean_source, source_indices):
        raise ValueError("Clean input indices differ from recorded run")
    rows = np.searchsorted(source_indices, candidate_source)
    if (not np.array_equal(source_indices[rows], candidate_source) or
            len(rows) != len(labels) or len(rows) != result["candidate_region_point_count"]):
        raise ValueError("Candidate indices/labels inconsistent")
    candidates = []
    for original in result["candidates"]:
        cid = int(original["id"])
        local = rows[labels == cid]
        xyz = analysis_xyz[local]
        if (not len(xyz) or not np.array_equal(source_indices[local], original["input_indices"]) or
                not np.allclose(xyz.mean(axis=0), original["center_xyz"])):
            raise ValueError(f"Candidate {cid} mismatches source indices or center")
        enriched = dict(original)
        enriched.update(xy_polar(original["center_xyz"], xyz, frame))
        enriched["obstacle_id"] = cid
        candidates.append(enriched)
    if len(candidates) != result["candidate_count"]:
        raise ValueError("Candidate count mismatch")
    source_ply = result_path.parent / "obstacles_colored.ply"
    cloud = o3d.io.read_point_cloud(str(source_ply))
    if (len(cloud.points) != len(rows) or len(cloud.colors) != len(rows) or
            not np.allclose(np.asarray(cloud.points), analysis_xyz[rows], atol=1e-5)):
        raise ValueError("R3 colored PLY differs from indexed analysis coordinates")
    output_dir.mkdir(parents=True, exist_ok=True)
    target_ply = output_dir / "obstacles_colored.ply"
    if source_ply.resolve() != target_ply.resolve():
        shutil.copyfile(source_ply, target_ply)
    plot = draw_radar(output_dir / "radar_view.png", candidates, analysis_xyz, rows, labels,
                      sample_name=source.parent.parent.name,
                      frame=frame, unit_note="m hypothesis / NPZ unit unverified",
                      calibration_note="+X vehicle-forward and height origin unverified",
                      max_radius=max_radius, ring_step=ring_step,
                      show_noise=show_noise, show_boxes=show_boxes)
    document = {
        "schema_version": 1,
        "result_type": "experimental_obstacle_candidates_with_xy_bearing",
        "sample_name": source.parent.parent.name,
        "source_result_json": str(result_path.resolve()),
        "input_file": str(source.resolve()), "input_sha256": result["input_sha256"],
        "source_fields": fields, "preprocess_stats": stats,
        "ground_method": result["ground_method"],
        "ground_parameters": result["parameters"]["ground"],
        "clustering_parameters": result["parameters"]["clustering"],
        "coordinate_parameters": result["parameters"]["coordinates"],
        "coordinate_frame": frame, "sensor_frame": result["sensor_frame"],
        "coordinate_unit": result["coordinate_unit"],
        "coordinate_unit_confirmed": result["coordinate_unit_confirmed"],
        "unit_note": "ROS driver meters are upstream; this NPZ export chain is unverified",
        "calibration_status": "experimental mounting hypothesis, not formally validated",
        "vehicle_forward_sensor_axis_verified": False,
        "mount_height_origin_verified": False,
        "angle_reference": ANGLE_REFERENCE,
        "range_xy_origin": "analysis-frame X=0,Y=0 (vertical shift does not alter XY)",
        "nearest_sensor_distance_reference": "raw livox_frame origin; 3-D legacy value",
        "azimuth_at_xy_origin": "0.0 by convention with azimuth_defined=false",
        "candidate_count": len(candidates),
        "unclustered_point_count": result["noise_point_count"],
        "candidates": candidates,
        "radar_plot": plot,
        "outputs": {"obstacles_json": str((output_dir / "obstacles.json").resolve()),
                    "radar_view_png": str((output_dir / "radar_view.png").resolve()),
                    "obstacles_colored_ply": str(target_ply.resolve())},
    }
    (output_dir / "obstacles.json").write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-radius", type=float)
    parser.add_argument("--ring-step", type=float)
    parser.add_argument("--hide-noise", action="store_true")
    parser.add_argument("--show-boxes", action="store_true")
    args = parser.parse_args()
    result = export_result(args.result_json, args.output_dir, max_radius=args.max_radius,
                           ring_step=args.ring_step, show_noise=not args.hide_noise,
                           show_boxes=args.show_boxes)
    print(json.dumps({"output": result["outputs"], "candidates": result["candidate_count"],
                      "plot_radius": result["radar_plot"]["plot_radius"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
