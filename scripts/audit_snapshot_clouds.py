"""Read-only structural and numeric audit for Snapshot NPZ point clouds."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from npz_to_ply import read_cloud


DEFAULT_DATA_ROOT = Path(r"E:\workspace\mower-data")


def discover_roots(data_root: Path, explicit_roots: list[Path]) -> list[Path]:
    candidates = explicit_roots or [
        data_root / "airport_mower_samples",
        data_root / "samples" / "airport_mower_samples",
    ]
    roots = [path for path in candidates if path.is_dir()]
    if not roots:
        raise FileNotFoundError(
            "找不到 Snapshot 根目录；请用 --sample-root 指定包含 sample_* 的目录"
        )
    return roots


def snapshot_dirs(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.glob("sample_*")
        if path.is_dir() and list((path / "lidar").glob("*.npz"))
    )


def allocate_snapshot_counts(counts: list[int], limit: int) -> list[int]:
    quotas = [min(count, limit // len(counts)) for count in counts]
    remaining = limit - sum(quotas)
    while remaining:
        changed = False
        for index, count in enumerate(counts):
            if quotas[index] < count and remaining:
                quotas[index] += 1
                remaining -= 1
                changed = True
        if not changed:
            break
    return quotas


def select_evenly(paths: list[Path], count: int) -> list[Path]:
    if count <= 0:
        return []
    indices = np.linspace(0, len(paths) - 1, min(count, len(paths)), dtype=int)
    return [paths[index] for index in indices]


def field_names(archive: np.lib.npyio.NpzFile) -> list[str]:
    if "fields" not in archive.files:
        return []
    return [str(value) for value in np.asarray(archive["fields"]).ravel()]


def intensity_values(archive: np.lib.npyio.NpzFile, names: list[str]):
    if "intensity" in archive.files:
        values = np.asarray(archive["intensity"])
        return values.ravel() if np.issubdtype(values.dtype, np.number) else None
    if "points" in archive.files:
        points = np.asarray(archive["points"])
        if points.ndim == 2 and "intensity" in names:
            return points[:, names.index("intensity")]
    return None


def axis_summary(points: np.ndarray) -> str:
    finite = points[np.isfinite(points).all(axis=1)]
    if finite.size == 0:
        return "无有限 XYZ 点"
    nonzero = finite[~np.all(finite == 0, axis=1)]
    low = finite.min(axis=0)
    high = finite.max(axis=0)
    if nonzero.size == 0:
        return f"XYZ min={low.tolist()} max={high.tolist()}；有限行均为全零 XYZ"
    p01, p50, p99 = np.percentile(nonzero, [1, 50, 99], axis=0)
    radius = np.linalg.norm(nonzero, axis=1)
    radial = np.percentile(radius, [0, 50, 95, 99, 100])
    zq = np.percentile(nonzero[:, 2], [0, 1, 5, 50, 95, 99, 100])
    return (
        f"finite rows={len(finite)} nonzero-XYZ rows={len(nonzero)} "
        f"XYZ min={np.round(low, 3).tolist()} max={np.round(high, 3).tolist()} "
        f"p01={np.round(p01, 3).tolist()} p50={np.round(p50, 3).tolist()} "
        f"p99={np.round(p99, 3).tolist()}\n"
        f"      range-from-origin (excluding all-zero XYZ rows) min/p50/p95/p99/max="
        f"{np.round(radial, 3).tolist()} ; "
        f"z-axis (excluding all-zero XYZ rows) p0/p1/p5/p50/p95/p99/p100="
        f"{np.round(zq, 3).tolist()}"
    )


def metadata_cloud_records(metadata: dict) -> dict[str, dict]:
    records = {}
    for record in metadata.get("clouds", []):
        name = Path(record.get("file", "")).name
        if name:
            records[name] = record
    return records


def audit_cloud(path: Path, record: dict, trigger_ns: int):
    # read_cloud is shared with npz_to_ply.py; this keeps XYZ interpretation
    # consistent between conversion and audit.
    xyz = np.asarray(read_cloud(path, verbose=False), dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f"XYZ 数组形状不正确: {xyz.shape}")

    with np.load(path, allow_pickle=False) as archive:
        names = field_names(archive)
        descriptions = []
        for name in archive.files:
            try:
                array = archive[name]
                details = f"{name}={array.shape}/{array.dtype}"
                if name == "fields":
                    details += f" values={np.asarray(array).ravel().tolist()}"
                descriptions.append(details)
            except ValueError as exc:
                descriptions.append(f"{name}=不可读取({exc})")
        intensity = intensity_values(archive, names)

    finite_rows = np.isfinite(xyz).all(axis=1)
    nonfinite_rows = int((~finite_rows).sum())
    zero_rows = int(np.all(xyz == 0, axis=1).sum())
    intensity_stats = "未找到 intensity 字段"
    if intensity is not None:
        finite_intensity = intensity[np.isfinite(intensity)]
        if finite_intensity.size:
            intensity_stats = (
                f"intensity dtype={intensity.dtype} finite="
                f"{finite_intensity.size}/{intensity.size} "
                f"min/p50/p99/max="
                f"{np.round(np.percentile(finite_intensity, [0, 50, 99, 100]), 3).tolist()}"
            )
        else:
            intensity_stats = f"intensity dtype={intensity.dtype}, 无有限值"

    count_match = record.get("point_count") in (None, len(xyz))
    header_ns = record.get("header_stamp_ns")
    receive_ns = record.get("receive_time_ns")
    time_info = f"frame={record.get('frame_id', '未记录')}"
    if header_ns is not None:
        time_info += f" header_stamp_ns={header_ns}"
    if receive_ns is not None:
        time_info += f" receive_time_ns={receive_ns}"
    if header_ns is not None and receive_ns is not None:
        time_info += f" receive-header_ms={(receive_ns - header_ns) / 1e6:.3f}"
    if receive_ns is not None and trigger_ns:
        time_info += f" receive-trigger_ms={(receive_ns - trigger_ns) / 1e6:.3f}"

    return xyz, (
        f"  {path.name}: {', '.join(descriptions)}\n"
        f"      rows={len(xyz)} metadata_count={record.get('point_count')} "
        f"count_match={count_match} nonfinite_xyz_rows={nonfinite_rows} "
        f"({100 * nonfinite_rows / max(len(xyz), 1):.5f}%) "
        f"all_zero_xyz_rows={zero_rows} ({100 * zero_rows / max(len(xyz), 1):.3f}%)\n"
        f"      {axis_summary(xyz)}\n"
        f"      {intensity_stats}\n"
        f"      {time_info}"
    )


def summarize_root(root: Path, selected: list[Path], clouds: list[np.ndarray]):
    if not clouds:
        return
    all_xyz = np.concatenate(clouds, axis=0)
    finite_rows = np.isfinite(all_xyz).all(axis=1)
    nonfinite_count = int((~finite_rows).sum())
    zero_count = int(np.all(all_xyz == 0, axis=1).sum())
    print(f"\n根目录汇总: {root}")
    print(
        f"  已抽查 Snapshot={len(selected)} NPZ={sum(len(list((p / 'lidar').glob('*.npz'))) for p in selected)} "
        f"点数={len(all_xyz)} 非有限 XYZ 行={nonfinite_count} "
        f"({100 * nonfinite_count / max(len(all_xyz), 1):.5f}%) "
        f"全零 XYZ 行={zero_count} ({100 * zero_count / max(len(all_xyz), 1):.3f}%)"
    )
    print(f"  {axis_summary(all_xyz)}")


def main():
    # PowerShell on Windows may otherwise send Python's Chinese text using the
    # legacy console code page, which makes the audit output unreadable.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    parser = argparse.ArgumentParser(
        description="只读抽查 Snapshot NPZ 点云；最多读取 10 个 sample_* 目录。"
    )
    parser.add_argument(
        "--data-root", type=Path, default=DEFAULT_DATA_ROOT,
        help="数据集父目录（默认检查 airport_mower_samples 和 samples/airport_mower_samples）",
    )
    parser.add_argument(
        "--sample-root", type=Path, action="append", default=[],
        help="包含 sample_* 目录的根目录；可重复指定以覆盖默认发现路径",
    )
    parser.add_argument(
        "--max-snapshots", type=int, choices=range(1, 11), default=10,
        help="最多抽查的 Snapshot 数量，范围 1..10（默认 10）",
    )
    args = parser.parse_args()

    roots = discover_roots(args.data_root, args.sample_root)
    available = [snapshot_dirs(root) for root in roots]
    counts = [len(paths) for paths in available]
    if not any(counts):
        raise FileNotFoundError("指定根目录下没有含 lidar/*.npz 的 sample_* 目录")
    quotas = allocate_snapshot_counts(counts, args.max_snapshots)
    selected_by_root = [select_evenly(paths, quota) for paths, quota in zip(available, quotas)]

    print("Snapshot NPZ 只读抽查")
    print(f"最多抽查: {args.max_snapshots} 组；实际抽查: {sum(map(len, selected_by_root))} 组")
    for root, paths, selected in zip(roots, available, selected_by_root):
        print(f"根目录 {root}: 可用 Snapshot={len(paths)}，抽查={[p.name for p in selected]}")

    for root, selected in zip(roots, selected_by_root):
        root_clouds = []
        for sample in selected:
            metadata_path = sample / "metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.is_file() else {}
            tf_path = sample / "tf_static.json"
            tf_data = json.loads(tf_path.read_text(encoding="utf-8")) if tf_path.is_file() else {}
            records = metadata_cloud_records(metadata)
            trigger_ns = int(metadata.get("trigger_receive_time_ns") or 0)

            print(
                f"\nSnapshot {sample.name}: session={metadata.get('session_id')} "
                f"created_utc={metadata.get('created_utc')} label_hint={metadata.get('label_hint')} "
                f"alignment_basis={metadata.get('alignment_basis')}"
            )
            print(f"  metadata frame_ids={[item.get('frame_id') for item in metadata.get('clouds', [])]}")
            transforms = tf_data.get("transforms", [])
            for transform in transforms:
                print(
                    f"  TF {transform.get('parent_frame_id')} -> {transform.get('child_frame_id')}: "
                    f"translation={transform.get('translation')} rotation={transform.get('rotation')} "
                    f"delta_from_trigger_ms={transform.get('delta_from_trigger_ms')}"
                )

            sample_clouds = []
            for path in sorted((sample / "lidar").glob("*.npz")):
                xyz, report = audit_cloud(path, records.get(path.name, {}), trigger_ns)
                print(report)
                sample_clouds.append(xyz)
            root_clouds.extend(sample_clouds)
        summarize_root(root, selected, root_clouds)


if __name__ == "__main__":
    main()
