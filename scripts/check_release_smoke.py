"""Exercise the public Core/Full CLI without requiring recorded field data."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
FIELD_CONFIG = ROOT / "configs/field_66cm_7p5deg_ransac_dbscan.json"


def main(output):
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Select a new output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(FIELD_CONFIG.read_text(encoding="utf-8"))
    coords = config["coordinates"]
    pitch = np.deg2rad(coords["mount_pitch_degrees"])
    cos, sin = np.cos(pitch), np.sin(pitch)

    # Generate a flat floor and two separated elevated returns in the analysis frame.
    x, y = np.meshgrid(np.linspace(1, 5, 31), np.linspace(-2, 2, 31))
    floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    rng = np.random.default_rng(312)
    targets = [rng.uniform(-.08, .08, (32, 3)) + center
               for center in ([2, -1, .7], [3, 1, .8])]
    x, y, z = np.vstack([floor, *targets]).T
    z = z - coords["mount_height"]
    sensor = np.column_stack((cos*x - sin*z, y, sin*x + cos*z, np.ones_like(x)))
    source = output / "synthetic.npz"
    np.savez_compressed(source, points=sensor.astype(np.float32),
                        fields=np.array(["x", "y", "z", "intensity"]))

    results = {}
    for mode in ("core", "full"):
        destination = output / mode
        subprocess.run([sys.executable, "-B", str(ROOT / "scripts/run_perception.py"),
                        "--input", str(source), "--config", str(FIELD_CONFIG),
                        "--mode", mode, "--output-dir", str(destination)], check=True)
        doc = results[mode] = json.loads((destination / "obstacles.json").read_text(encoding="utf-8"))
        assert doc["coordinate_parameters"] == coords
        assert doc["clustering_parameters"] == config["clustering"]
        assert doc["candidate_count"] == 2
        assert sorted(c["point_count"] for c in doc["candidates"]) == [32, 32]
        assert all(c["azimuth_defined"] for c in doc["candidates"])
    assert results["core"]["removed_point_count"] == len(floor)
    assert results["core"]["selected_plane_diagnostics"] is not None
    assert {p.name for p in (output / "core").iterdir()} == {"obstacles.json"}
    assert results["core"]["radar_plot"] is None
    for core, full in zip(results["core"]["candidates"], results["full"]["candidates"], strict=True):
        assert core["point_cloud"] is None
        assert {k: v for k, v in core.items() if k != "point_cloud"} == {
            k: v for k, v in full.items() if k != "point_cloud"}
    for name in ("ground.ply", "nonground.ply", "obstacles_colored.ply"):
        assert (output / "full" / name).stat().st_size > 0
    with Image.open(output / "full/radar_view.png") as png:
        png.verify()
    print("Release smoke passed: two candidates, matching Core/Full geometry, Full PLY and PNG.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    main(parser.parse_args().output_dir)
