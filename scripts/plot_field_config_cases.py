"""Plot the two R3.10 cases at eps=.35 and check same-point connectivity."""

import argparse
from pathlib import Path

from check_field_config import CASES, FIELD_CONFIG, HISTORY, DEFAULT_CONFIG, dump, load
from offline_detector import analysis_coordinates, run
from analyze_field_merging import core_path
from extract_obstacle_candidates import candidate_color

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree


def draw(path, xyz, field_labels, old_labels, old_label, new_label, low, high):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), layout="constrained")
    for column, (labels, focus, track, title) in enumerate((
            (old_labels, old_label, low, "Old mount: retained low-plane connection"),
            (field_labels, new_label, high, "Field mount: raised core-point path"),
    )):
        for row, (j, limits) in enumerate(((1, (-1.8, 3.1)), (2, (-.08, 3.1)))):
            ax = axes[row, column]
            for mask, color, size, opacity in (
                    (labels == -2, "#609260", 5, .45),
                    (labels == -1, "#a0a0a0", 4, .42),
                    ((labels >= 0) & (labels != focus), "#5e7899", 7, .55),
                    (labels == focus, candidate_color(focus), 9, .75)):
                ax.scatter(xyz[mask, 0], xyz[mask, j], s=size, color=color, alpha=opacity,
                           linewidths=0, rasterized=True)
            if len(track):
                ax.plot(xyz[track, 0], xyz[track, j], "-o", c="#191a1d", lw=1.4, ms=3, zorder=6)
                if column == 0:
                    cut = track[field_labels[track] == -2]
                    ax.scatter(xyz[cut, 0], xyz[cut, j], s=32, facecolors="#fff9db",
                               edgecolors="#191a1d", zorder=7)
            ax.set(xlim=(.4, 4), ylim=limits, xlabel="X (field analysis units)",
                   ylabel="XYZ"[j] + " (field analysis units)")
            ax.set_aspect("equal")
            ax.grid(alpha=.18)
        axes[0, column].set_title(f"{title}\nFocused label #{focus}, eps = 0.35")
    fig.suptitle("Both panels shown in field coordinates\n"
                 "Green: ground candidate · gray: noise · blue: other clusters · black: core path\n"
                 "Hollow nodes on the left become ground candidates with the field mount")
    fig.savefig(path, dpi=155)
    plt.close(fig)
    with Image.open(path) as im:
        im.verify()


def inspect(output, bag, mid):
    source = HISTORY / "converted" / bag / f"message_{mid:06d}.npz"
    folder = output / "full" / bag / source.stem
    case_folder = output / "cases" / f"{bag}__{mid:06d}"
    case_folder.mkdir(parents=True)
    doc = load(folder / "detection/result.json")
    with np.load(folder / "detection/indices_and_labels.npz") as data:
        clean = data["clean_source_indices"]
        rows = np.searchsorted(clean, data["candidate_source_indices"])
        labels = data["candidate_labels"]
    with np.load(source) as data:
        xyz, _ = analysis_coordinates(data["points"][clean, :3], doc["parameters"]["coordinates"])
    field = np.full(len(xyz), -2, dtype=int)
    field[rows] = labels
    archive_path = HISTORY / "cases" / f"{bag}__{mid:06d}" / "connectivity_arrays.npz"
    # Historical .25 labels only identify endpoints. Every path is measured at the current .35 radius.
    with np.load(archive_path) as archived:
        assert np.array_equal(archived["points"], xyz[rows])
        assert np.array_equal(archived["eps_035"], labels)
        historical_labels = archived["eps_025"]
    old = {}
    old_result, _ = run(source, DEFAULT_CONFIG, output, "unused", export_artifacts=False, context=old)
    assert old_result["removed_point_count"] == 0
    assert np.array_equal(old["candidate_rows"], np.arange(len(xyz)))
    old_labels = old["labels"]
    # The endpoints were picked from near-field plots and same-point label overlap.
    old_id, main_id, other_id = (3, 3, 21) if mid == 92 else (7, 7, 25)
    near = (xyz[:, 0] >= .4) & (xyz[:, 0] <= 3) & (np.abs(xyz[:, 1]) <= 2)
    old_mask = old_labels == old_id
    low = core_path(xyz, old_labels, near & old_mask & (field == main_id),
                    near & old_mask & (field == other_id), .35)
    focused = (labels == main_id) & near[rows]
    branches, sizes = np.unique(historical_labels[focused & (historical_labels >= 0)], return_counts=True)
    main, second = branches[np.argsort(-sizes)[:2]]
    high = rows[core_path(xyz[rows], labels, focused & (historical_labels == main),
                         focused & (historical_labels == second), .35)]
    plane = np.asarray(doc["plane_model_ax_by_cz_d"])
    distance = (xyz @ plane[:3] + plane[3]) / np.linalg.norm(plane[:3])
    # Geometric proximity only: footprints may legitimately overlap the ground.
    bottom = xyz[near & (field == main_id) & (np.abs(distance) <= .30), :2]
    ground = np.flatnonzero((field == -2) & near)
    foot = cKDTree(bottom).query(xyz[ground, :2])[0] <= .12
    assessed = ground[foot]
    ids, counts = np.unique(field[old_mask], return_counts=True)
    assert np.any(field[low] == -2) and np.all(np.abs(distance[high]) > .12)
    measures = {"message_id": mid, "bag": bag, "eps": .35, "old_label": old_id,
                "field_label": main_id, "other_endpoint_field_label": other_id,
                "old_label_full_scene_partition": dict(zip(map(str, ids), map(int, counts))),
                "old_path_source_point_indices": clean[low].tolist(),
                "old_path_nodes_now_ground": int(np.count_nonzero(field[low] == -2)),
                "high_path_source_point_indices": clean[high].tolist(),
                "high_path_signed_distances": distance[high].tolist(),
                "low_path_field_signed_distances": distance[low].tolist(),
                "high_path_edge_lengths": np.linalg.norm(np.diff(xyz[high], axis=0), axis=1).tolist(),
                "ground_candidates_within_xy_012_of_low_obstacle_points": int(len(assessed)),
                "near_low_obstacle_points_used": int(len(bottom)),
                "definitions": {"-2": "ground candidate", "-1": "noise",
                                "proximity": "near window, retained focus points within 0.30 of the plane; XY <=0.12"}}
    dump(case_folder / "geometry_checks.json", measures)
    draw(case_folder / "connection_check.png", xyz, field, old_labels, old_id, main_id, low, high)
    return measures


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for bag, mid in CASES:
        print(inspect(args.output, bag, mid))
