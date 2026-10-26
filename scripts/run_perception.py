"""Run the reviewed experimental RANSAC + DBSCAN + radar-view pipeline."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from offline_detector import run
from radar_view import export_result


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "configs" / "r3_2_ransac_normal_trial.json"


def run_pipeline(input_path: Path, output_dir: Path, config_path: Path = DEFAULT_CONFIG):
    """Compose existing modules without changing their processing or JSON fields."""
    input_path, output_dir, config_path = map(Path, (input_path, output_dir, config_path))
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("ground", {}).get("method") != "ransac_normal":
        raise ValueError("Main pipeline requires ground.method=ransac_normal")
    if config.get("clustering", {}).get("method") != "dbscan":
        raise ValueError("Main pipeline requires clustering.method=dbscan")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output_dir}")
    result, detection_dir = run(input_path, config_path, output_dir, "detection")
    # The exporter validates this colored cloud against source row indices.
    shutil.copyfile(detection_dir / "clusters_colored.ply", detection_dir / "obstacles_colored.ply")
    layers = {
        "removed_experimental.ply": "ground.ply",
        "candidate_region.ply": "nonground.ply",
    }
    for source_name, target_name in layers.items():
        source = detection_dir / source_name
        if source.is_file():
            shutil.copyfile(source, output_dir / target_name)
    enriched = export_result(detection_dir / "result.json", output_dir)
    return result, enriched


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Snapshot NPZ file")
    parser.add_argument("--output-dir", required=True, type=Path, help="New empty output directory")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    result, enriched = run_pipeline(args.input, args.output_dir, args.config)
    print(json.dumps({
        "output_dir": str(args.output_dir.resolve()),
        "candidate_count": result["candidate_count"],
        "remaining_point_count": result["preprocess_stats"]["remaining_point_count"],
        "outputs": enriched["outputs"],
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
