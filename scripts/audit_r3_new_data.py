"""Read-only inventory of the R3.1 Snapshot directory; outputs a local manifest."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from npz_to_ply import read_cloud_data
from prepare_cloud import filter_point_rows


def utc_from_ns(value):
    if value is None:
        return None
    return datetime.fromtimestamp(int(value) / 1e9, timezone.utc).isoformat()


def audit(root: Path, output_dir: Path):
    snapshots = sorted(p for p in root.iterdir() if p.is_dir())
    if not snapshots:
        raise ValueError(f"No Snapshot directories: {root}")
    output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    image_cards = []
    for folder in snapshots:
        metadata_path = folder / "metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.is_file() else {}
        cloud_meta = {Path(c.get("file", "")).name: c for c in metadata.get("clouds", [])}
        image_path = folder / "images" / "image_00.png"
        if image_path.is_file():
            with Image.open(image_path) as src:
                thumbnail = src.convert("RGB")
                thumbnail.thumbnail((300, 220))
                card = Image.new("RGB", (320, 250), "white")
                card.paste(thumbnail, ((320 - thumbnail.width) // 2, 5))
                ImageDraw.Draw(card).text((8, 228), folder.name, fill="black")
                image_cards.append(card)
        for npz in sorted(folder.rglob("*.npz")):
            with np.load(npz, allow_pickle=False) as data:
                schema = {name: {"shape": list(data[name].shape), "dtype": str(data[name].dtype)}
                          for name in data.files}
            xyz, intensity, fields = read_cloud_data(npz, verbose=False)
            xyz = np.asarray(xyz)
            keep, stats = filter_point_rows(xyz, intensity, "exclude")
            clean = xyz[keep].astype(np.float64)
            radial = np.linalg.norm(clean, axis=1) if len(clean) else np.empty(0)
            intensity_values = np.asarray(intensity).reshape(-1) if intensity is not None else np.empty(0)
            finite_intensity = intensity_values[np.isfinite(intensity_values)]
            cloud_info = cloud_meta.get(npz.name, {})
            records.append({
                "file": str(npz.resolve()), "snapshot": folder.name,
                "schema": schema, "fields": fields, "readable_by_existing_parser": True,
                "preprocess": stats,
                "xyz_min": clean.min(axis=0).tolist() if len(clean) else None,
                "xyz_max": clean.max(axis=0).tolist() if len(clean) else None,
                "radial_quantiles_raw_values": np.quantile(radial, [0, .5, .9, .99, 1]).tolist() if len(clean) else None,
                "intensity_min_max": [float(finite_intensity.min()), float(finite_intensity.max())]
                if len(finite_intensity) else None,
                "intensity_nonzero_ratio": float((finite_intensity != 0).mean()) if len(finite_intensity) else None,
                "metadata_point_count": cloud_info.get("point_count"),
                "frame_id": cloud_info.get("frame_id"),
                "header_stamp_ns": cloud_info.get("header_stamp_ns"),
                "header_utc": utc_from_ns(cloud_info.get("header_stamp_ns")),
                "receive_time_ns": cloud_info.get("receive_time_ns"),
                "receive_utc": utc_from_ns(cloud_info.get("receive_time_ns")),
                "session_id": metadata.get("session_id"),
                "created_utc": metadata.get("created_utc"),
                "label_hint": metadata.get("label_hint"),
                "scene_hint": metadata.get("scene_hint"),
                "operator_note": metadata.get("operator_note"),
            })
    columns = 5
    rows = (len(image_cards) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * 320, rows * 250), "#eeeeee")
    for i, card in enumerate(image_cards):
        sheet.paste(card, ((i % columns) * 320, (i // columns) * 250))
    sheet.save(output_dir / "snapshot_image_contact_sheet.png")
    summary = {
        "root": str(root.resolve()), "snapshot_count": len(snapshots),
        "npz_count": len(records), "image_count": len(image_cards),
        "field_layouts": dict(Counter(tuple(r["fields"]) for r in records)),
        "sessions": dict(Counter(r["session_id"] or "UNKNOWN" for r in records)),
        "scene_hints": dict(Counter(r["scene_hint"] or "UNKNOWN" for r in records)),
        "label_hints": dict(Counter(r["label_hint"] or "UNKNOWN" for r in records)),
        "records": records,
    }
    # JSON keys must be strings; field layout counts are clearer as records.
    summary["field_layouts"] = [
        {"fields": list(fields), "count": count}
        for fields, count in Counter(tuple(r["fields"]) for r in records).items()
    ]
    (output_dir / "newdata_manifest.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps({k: summary[k] for k in ("snapshot_count", "npz_count", "image_count", "field_layouts", "sessions", "scene_hints", "label_hints")}, ensure_ascii=False))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(r"E:\workspace\mower-data\data\newdata"))
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent.parent / "outputs" / "r3_new_data_trial")
    args = parser.parse_args()
    audit(args.root, args.output_dir)


if __name__ == "__main__":
    main()
