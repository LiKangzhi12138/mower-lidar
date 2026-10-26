"""Read-only integrity checks for the seven R3.3 offline trial outputs."""

import json
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image

from extract_obstacle_candidates import sha256_file


ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "outputs" / "r3_multiscene_validation"
DATA = Path(r"E:\workspace\mower-data\data\newdata")
METHODS = ("ransac_normal", "patchwork")


def check_image(path):
    with Image.open(path) as image:
        image.verify()


def check_cloud(path, count):
    if count == 0:
        assert not path.exists(), path
        return
    cloud = o3d.io.read_point_cloud(str(path))
    assert len(cloud.points) == len(cloud.colors) == count, path
    assert np.isfinite(np.asarray(cloud.points)).all(), path


def main():
    summary = json.loads((OUT / "validation_summary.json").read_text(encoding="utf-8"))
    assert summary["sample_ids"] == [f"sample_{n:06d}" for n in range(143, 150)]
    assert len(summary["records"]) == 7
    assert not any("sample_000142" in r["source"] for r in summary["records"])
    check_image(OUT / "seven_sample_comparison.png")
    ply_count = 0
    candidate_ply_count = 0
    png_count = 1
    for record in summary["records"]:
        sample = record["sample"]
        source = DATA / sample / "lidar" / "cloud_01.npz"
        assert record["input_sha256"] == sha256_file(source)
        per_sample = json.loads((OUT / sample / "summary.json").read_text(encoding="utf-8"))
        clean_count = per_sample["preprocess"]["remaining_point_count"]
        assert record["clean_points"] == clean_count
        check_image(OUT / sample / "patchwork_vs_ransac.png")
        png_count += 1
        clean_indices = None
        for method in METHODS:
            folder = OUT / sample / method
            result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
            config = ROOT / "configs" / f"r3_2_{'ransac_normal' if method == 'ransac_normal' else 'patchwork'}_trial.json"
            assert result["parameters"] == json.loads(config.read_text(encoding="utf-8"))
            assert result["input_sha256"] == record["input_sha256"]
            with np.load(folder / "indices_and_labels.npz") as mapping:
                clean = mapping["clean_source_indices"]
                candidate = mapping["candidate_source_indices"]
                ground = mapping["removed_source_indices"]
                unclassified = mapping["unclassified_source_indices"]
                labels = mapping["candidate_labels"]
            if clean_indices is None:
                clean_indices = clean
            else:
                assert np.array_equal(clean_indices, clean)
            assert len(clean) == clean_count
            assert not np.intersect1d(candidate, ground).size
            assert np.array_equal(np.sort(np.concatenate((candidate, ground))), clean)
            assert len(ground) + len(candidate) == clean_count
            assert len(unclassified) == result["unclassified_point_count"]
            assert len(labels) == len(candidate)
            assert np.count_nonzero(labels < 0) == result["noise_point_count"]
            assert len(np.unique(labels[labels >= 0])) == result["candidate_count"]
            nonground = len(candidate) - len(unclassified)
            for name, count in (("original.ply", clean_count), ("ground.ply", len(ground)),
                                ("nonground.ply", nonground),
                                ("obstacles_colored.ply", len(candidate)),
                                ("unclassified.ply", len(unclassified)),
                                ("unclustered.ply", result["noise_point_count"])):
                check_cloud(folder / name, count)
                ply_count += int(count > 0)
            for item in result["candidates"]:
                check_cloud(folder / item["point_cloud"], item["point_count"])
                candidate_ply_count += 1
            check_image(folder / "overview.png")
            png_count += 1
    print(json.dumps({"samples": 7, "runs": 14, "required_nonempty_ply_checked": ply_count,
                      "candidate_ply_checked": candidate_ply_count,
                      "png_checked": png_count, "status": "passed"}))


if __name__ == "__main__":
    main()
