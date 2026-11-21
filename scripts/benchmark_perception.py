"""R3.8: measure existing entry points; never change algorithm parameters."""
from __future__ import annotations
import argparse
import ctypes
import hashlib
import json
import platform
import subprocess
import sys
import threading
import time
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(r"E:\workspace\mower-data\data\newdata")


def rss():
    class Counters(ctypes.Structure):
        _fields_ = [("cb", ctypes.c_ulong), ("PageFaultCount", ctypes.c_ulong)] + [
            (name, ctypes.c_size_t) for name in ("PeakWorkingSetSize", "WorkingSetSize",
            "QuotaPeakPagedPoolUsage", "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage",
            "QuotaNonPagedPoolUsage", "PagefileUsage", "PeakPagefileUsage")]
    value = Counters(); value.cb = ctypes.sizeof(value)
    kernel = ctypes.WinDLL("kernel32"); kernel.GetCurrentProcess.restype = ctypes.c_void_p
    query = ctypes.WinDLL("psapi").GetProcessMemoryInfo
    query.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
    if not query(kernel.GetCurrentProcess(), ctypes.byref(value), value.cb):
        raise ctypes.WinError()
    return value.WorkingSetSize


def measure(mode, source, output):
    if mode == "baseline":
        # 原始版本快照仅用于基线复核，避免后续源码变更污染原始测量。
        snapshot = ROOT / "outputs/r3_performance/original_source"
        if not snapshot.exists():
            raise FileNotFoundError("Baseline requires the frozen prechange original_source snapshot")
        if str(snapshot) not in sys.path:
            sys.path.insert(0, str(snapshot))
    import run_perception as entry
    import offline_detector as detector
    import radar_view as radar
    from matplotlib.figure import Figure
    from contextlib import ExitStack
    stages, calls = {}, {}
    def wrapper(fn, key):
        def timed(*args, **kwargs):
            started = time.perf_counter_ns()
            try:
                return fn(*args, **kwargs)
            finally:
                stages[key] = stages.get(key, 0) + time.perf_counter_ns() - started
                calls[key] = calls.get(key, 0) + 1
        return timed
    stop = threading.Event(); memory = [rss()]
    def sample():
        while not stop.wait(0.005): memory.append(rss())
    thread = threading.Thread(target=sample, daemon=True); thread.start()
    # 只包装函数边界；嵌套计时分别记录，不能直接相加。
    with ExitStack() as stack:
        targets = [(detector, "read_cloud_data", "npz_read"),
            (detector, "filter_point_rows", "filter"), (detector, "analysis_coordinates", "transform"),
            (detector, "normal_constrained_plane", "ransac"), (detector, "cluster_dbscan", "dbscan"),
            (detector, "describe_candidate", "geometry"), (detector, "_write_ply", "ply_write"),
            (detector, "sha256_file", "initial_hash"), (radar, "read_cloud_data", "repeat_npz_read"),
            (radar, "filter_point_rows", "repeat_filter"), (radar, "analysis_coordinates", "repeat_transform"),
            (radar, "sha256_file", "repeat_hash"), (radar, "xy_polar", "xy_geometry"),
            (radar, "draw_radar", "radar_inclusive"), (Figure, "savefig", "png_save"),
            (radar.o3d.io, "read_point_cloud", "ply_validate_read"),
            (entry.shutil, "copyfile", "file_copy"), (json, "dumps", "json_serialize"),
            (radar.np, "allclose", "consistency_checks"), (radar.np, "array_equal", "consistency_checks")]
        for obj, name, key in targets:
            stack.enter_context(patch.object(obj, name, wrapper(getattr(obj, name), key)))
        original_write = Path.write_text
        def write_text(path, *args, **kwargs):
            return wrapper(original_write, "json_write")(path, *args, **kwargs)
        stack.enter_context(patch.object(Path, "write_text", write_text))
        cpu = time.process_time_ns(); started = time.perf_counter_ns()
        if mode == "baseline":
            result, document = entry.run_pipeline(source, output, ROOT / "configs/r3_2_ransac_normal_trial.json")
        else: result, document = entry.run_pipeline(source, output, mode=mode)
        elapsed = time.perf_counter_ns() - started; cpu = time.process_time_ns() - cpu
    memory.append(rss()); stop.set(); thread.join()
    files = list(output.rglob("*")); files = [p for p in files if p.is_file()]
    stages["radar_draw_excluding_save"] = stages.get("radar_inclusive", 0) - stages.get("png_save", 0)
    return {"mode": mode, "sample": source.parent.parent.name, "wall_ns": elapsed,
        "cpu_ns": cpu, "sampled_peak_working_set_bytes": max(memory), "rss_before_bytes": memory[0],
        "stages_ns": stages, "calls": calls, "files": len(files), "bytes": sum(p.stat().st_size for p in files),
        "raw_points": result["preprocess_stats"]["input_point_count"],
        "clean_points": result["preprocess_stats"]["remaining_point_count"],
        "nonground_points": result["candidate_region_point_count"],
        "candidates": result["candidate_count"], "noise_points": result["noise_point_count"],
        "geometry_sha256": hashlib.sha256(json.dumps(document["candidates"], sort_keys=True).encode()).hexdigest()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["baseline", "full", "core"], required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--cold-worker", type=int)
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise FileExistsError(f"Use a new empty benchmark directory: {args.output_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.cold_worker is not None:
        row = measure(args.mode, DATA / f"sample_{args.cold_worker:06d}" / "lidar/cloud_01.npz", args.output_dir / "files")
        (args.output_dir / "measurement.json").write_text(json.dumps(row), encoding="utf-8")
        return
    metadata = {"python": sys.version, "platform": platform.platform(), "cpu": platform.processor(),
        "repeats_per_frame": args.repeats, "warmup_per_frame": 1, "memory_sampling_ms": 5,
        "source_hashes": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT / "scripts").glob("*.py")},
        "config_sha256": hashlib.sha256((ROOT / "configs/r3_2_ransac_normal_trial.json").read_bytes()).hexdigest()}
    executed = ROOT / ("outputs/r3_performance/original_source" if args.mode == "baseline" else "scripts")
    metadata["executed_source_hashes"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in executed.glob("*.py")}
    (args.output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    rows=[]; cold=[]
    for number in range(142, 150):
        folder = args.output_dir / f"cold_{number}"
        started = time.perf_counter_ns()
        subprocess.run([sys.executable, "-B", __file__, "--mode", args.mode, "--output-dir", str(folder),
            "--cold-worker", str(number)], check=True, stdout=subprocess.DEVNULL)
        process_elapsed = time.perf_counter_ns()-started
        row=json.loads((folder/"measurement.json").read_text()); row["process_wall_ns"]=process_elapsed; cold.append(row)
    for number in range(142, 150):
        source=DATA/f"sample_{number:06d}"/"lidar/cloud_01.npz"
        measure(args.mode, source, args.output_dir/f"warmup_{number}")
        for repeat in range(args.repeats):
            row=measure(args.mode, source, args.output_dir/f"run_{number}_{repeat:02d}")
            row["repeat"]=repeat; rows.append(row)
        print(args.mode, number, "done", flush=True)
    (args.output_dir/"measurements.json").write_text(json.dumps({"cold":cold,"warm":rows},indent=2),encoding="utf-8")


if __name__ == "__main__": main()
