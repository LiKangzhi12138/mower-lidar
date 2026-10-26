"""Verify R3.1 output correspondence against only the selected newdata NPZ files."""

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image

from npz_to_ply import read_cloud_data
from prepare_cloud import filter_point_rows
from offline_detector import ROOT


def cloud_points(path, expected):
    if expected == 0:
        assert not path.exists(), path
        return np.empty((0, 3))
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    assert len(points) == expected and len(cloud.colors) == expected, path
    return points


def check(output_dir):
    trial = json.loads((output_dir / "trial_summary.json").read_text(encoding="utf-8"))
    selected = trial["samples"]
    assert len(selected) == 12
    runs_checked = 0
    clouds_checked = 0
    figures_checked = 0
    limits = np.asarray(trial["fixed_plot_axis_limits_raw_values"])
    clipped = total = 0
    for record in selected:
        source = Path(record["input_file"])
        assert "newdata" in source.parts and source.is_file()
        raw, intensity, fields = read_cloud_data(source, verbose=False)
        raw = np.asarray(raw)
        keep, _ = filter_point_rows(raw, intensity, "exclude")
        clean_source = np.flatnonzero(keep)
        clean = raw[keep].astype(np.float64)
        total += len(clean)
        clipped += int((~((clean >= limits[:, 0]) & (clean <= limits[:, 1])).all(axis=1)).sum())
        assert len(clean) == record["clean_points"]
        with Image.open(record["comparison_figure"]) as picture:
            picture.verify()
        figures_checked += 1
        hashes = set()
        for method, directory in record["run_directories"].items():
            folder = Path(directory)
            result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
            assert Path(result["input_file"]) == source
            assert result["coordinate_unit_confirmed"] is False
            assert result["analysis_frame"] == "livox_frame"
            assert result["preprocess_stats"]["remaining_point_count"] == len(clean)
            hashes.add(result["input_sha256"])
            with np.load(folder / "indices_and_labels.npz") as mapped:
                assert np.array_equal(mapped["clean_source_indices"], clean_source)
                assert np.array_equal(mapped["candidate_source_indices"], clean_source)
                assert len(mapped["removed_source_indices"]) == 0
                labels = mapped["candidate_labels"]
                plane_source = mapped["plane_source_indices"]
            assert len(labels) == len(clean)
            assert int((labels < 0).sum()) == result["noise_point_count"]
            region = cloud_points(folder / "candidate_region.ply", len(clean))
            assert np.allclose(region, clean, rtol=0, atol=1e-5)
            clouds_checked += 1
            cloud_points(folder / "clusters_colored.ply", len(clean))
            clouds_checked += 1
            cloud_points(folder / "plane_candidate.ply", len(plane_source))
            if len(plane_source):
                clouds_checked += 1
            assert result["plane_candidate_point_count"] == len(plane_source)
            if method == "ransac_plane":
                assert result["candidate_count"] == 0
                assert len(plane_source) > 0
            else:
                assert len(plane_source) == 0
            clustered = 0
            for candidate in result["candidates"]:
                indices = np.asarray(candidate["input_indices"], dtype=np.int64)
                points = raw[indices].astype(np.float64)
                assert len(points) == candidate["point_count"]
                assert np.allclose(points.mean(axis=0), candidate["center_xyz"])
                assert np.allclose(points.min(axis=0), candidate["bbox_min_xyz"])
                assert np.allclose(points.max(axis=0), candidate["bbox_max_xyz"])
                assert np.isclose(np.linalg.norm(points, axis=1).min(),
                                  candidate["nearest_sensor_distance"])
                exported = cloud_points(folder / candidate["point_cloud"], len(points))
                assert np.allclose(exported, points, rtol=0, atol=1e-5)
                clustered += len(points)
                clouds_checked += 1
            assert clustered + result["noise_point_count"] == len(clean)
            runs_checked += 1
        assert len(hashes) == 1
    result = {"selected_frames": len(selected), "method_runs": runs_checked,
              "ply_files_read": clouds_checked, "comparison_png_read": figures_checked,
              "clean_points_outside_common_plot_limits": clipped,
              "selected_clean_points": total,
              "outside_fraction": clipped / total if total else 0}
    (output_dir / "validation_summary.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "r3_new_data_trial")
    args = parser.parse_args()
    check(args.output_dir)


if __name__ == "__main__":
    main()
