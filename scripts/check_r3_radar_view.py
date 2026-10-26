"""Check XY bearing conventions and existing R3.4 JSON/PNG/PLY outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image

from extract_obstacle_candidates import candidate_color, sha256_file
from npz_to_ply import read_cloud_data
from offline_detector import analysis_coordinates
from prepare_cloud import filter_point_rows
from radar_view import ANGLE_REFERENCE, choose_rings, display_xy, xy_polar


ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "outputs" / "r3_radar_view"


def check_geometry():
    for xyz, expected in (((1, 0, 0), 0), ((0, 1, 0), 90),
                          ((0, -1, 0), -90), ((-1, 0, 0), 180)):
        got = xy_polar(xyz, np.asarray([xyz]), "test")
        assert np.isclose(got["azimuth_deg"], expected)
        assert np.isclose(got["range_xy"], 1)
        assert got["azimuth_defined"]
    zero = xy_polar((0, 0, 3), np.asarray([[0, 0, 3]]), "test")
    assert zero["range_xy"] == zero["azimuth_deg"] == zero["nearest_range_xy"] == 0
    assert not zero["azimuth_defined"]
    distinct = xy_polar((2, 0, 0), np.asarray([[1, 0, 20], [2, 0, 0]]), "test")
    assert distinct["range_xy"] == 2 and distinct["nearest_range_xy"] == 1
    for direction, screen in (((1, 0), (0, 1)), ((0, 1), (-1, 0)),
                              ((0, -1), (1, 0)), ((-1, 0), (0, -1))):
        assert np.array_equal(display_xy(direction), screen)
    samples = np.array([[2.0, -3.0], [-4.0, 5.0]])
    assert np.allclose(np.linalg.norm(display_xy(samples), axis=1), np.linalg.norm(samples, axis=1))
    radius, rings = choose_rings(13.12)
    assert radius >= 13.12 and np.all(np.diff(rings) > 0) and np.all(rings <= radius)
    radius, rings = choose_rings(3.0, max_radius=8.0, ring_step=2.0)
    assert radius == 8.0 and np.array_equal(rings, [2.0, 4.0, 6.0, 8.0])


def check_sample(number):
    name = f"sample_{number:06d}"
    folder = OUT / name
    doc = json.loads((folder / "obstacles.json").read_text(encoding="utf-8"))
    reference_path = Path(doc["source_result_json"])
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    assert doc["input_sha256"] == reference["input_sha256"] == sha256_file(Path(doc["input_file"]))
    assert doc["candidate_count"] == reference["candidate_count"] == len(doc["candidates"])
    assert len({c["id"] for c in doc["candidates"]}) == len(doc["candidates"])
    assert doc["sample_name"] == name
    assert doc["coordinate_frame"] == reference["analysis_frame"]
    assert doc["coordinate_unit_confirmed"] is False
    assert doc["angle_reference"] == ANGLE_REFERENCE
    raw, intensity, _ = read_cloud_data(Path(doc["input_file"]), verbose=False)
    keep, _ = filter_point_rows(raw, intensity, reference["parameters"]["preprocess"]["zero_policy"])
    source_idx = np.flatnonzero(keep)
    xyz, frame = analysis_coordinates(np.asarray(raw)[keep], reference["parameters"]["coordinates"])
    assert frame == doc["coordinate_frame"]
    with np.load(reference_path.parent / "indices_and_labels.npz") as mapped:
        source_rows = mapped["candidate_source_indices"]
        labels = mapped["candidate_labels"]
    rows = np.searchsorted(source_idx, source_rows)
    assert np.array_equal(source_idx[rows], source_rows)
    cloud = o3d.io.read_point_cloud(str(folder / "obstacles_colored.ply"))
    assert len(cloud.points) == len(rows) == len(cloud.colors)
    assert np.allclose(np.asarray(cloud.points), xyz[rows], atol=1e-5)
    rgb = np.asarray(cloud.colors)
    for original, candidate in zip(reference["candidates"], doc["candidates"]):
        cid = original["id"]
        assert candidate["id"] == candidate["obstacle_id"] == cid
        for key, value in original.items():
            assert candidate[key] == value, (name, cid, key)
        local = rows[labels == cid]
        points = xyz[local]
        assert len(local) == candidate["point_count"]
        assert np.allclose(rgb[labels == cid], candidate_color(cid), atol=1 / 255 + 1e-5)
        assert np.isclose(candidate["range_xy"], np.hypot(*candidate["center_xyz"][:2]))
        assert np.isclose(candidate["nearest_range_xy"], np.min(np.linalg.norm(points[:, :2], axis=1)))
        assert np.isclose(candidate["azimuth_deg"], np.degrees(np.arctan2(
            candidate["center_xyz"][1], candidate["center_xyz"][0])))
        assert candidate["coordinate_frame"] == frame
        assert candidate["angle_reference"] == ANGLE_REFERENCE
    with Image.open(folder / "radar_view.png") as image:
        assert image.width >= 1200 and image.height >= 900
        image.verify()
    plot = doc["radar_plot"]
    assert plot["equal_xy_scale"] is True
    assert plot["clustered_points_outside_plot"] == 0
    assert plot["display_mapping"] == "screen_horizontal=-analysis_Y; screen_vertical=analysis_X"
    assert all(0 < ring <= plot["plot_radius"] for ring in plot["ring_radii"])
    assert plot["unclustered_points_total"] == reference["noise_point_count"]
    return {"sample": name, "candidates": len(doc["candidates"]),
            "plot_radius": plot["plot_radius"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", nargs="+", type=int, default=[142])
    args = parser.parse_args()
    check_geometry()
    report = [check_sample(n) for n in args.samples]
    print(json.dumps({"status": "passed", "geometry_cases": 8, "samples": report}))


if __name__ == "__main__":
    main()
