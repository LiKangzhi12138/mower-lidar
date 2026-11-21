"""Read the field bags' standard PointCloud2 CDR1 messages into lossless NPZ."""

import argparse
from contextlib import closing
import hashlib
from io import BytesIO
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np

from npz_to_ply import read_cloud_data

BAGS = {
    "front_1m_20261119-164850": (27, (80, 133, 184)),
    "front_1m_20261119-165004": (51, (5, 144, 284)),
    "partial_overlap_20261119-165259": (33, (15, 92, 179)),
}
TOPIC = "/livox/lidar"
FIELDS = ("x", "y", "z", "intensity")
TYPES = dict(zip(range(1, 9), ("i1", "u1", "i2", "u2", "i4", "u4", "f4", "f8")))
REQUIRED = dict(zip((*FIELDS, "tag", "line", "timestamp"), (7, 7, 7, 7, 2, 2, 8)))


def open_bag(path):
    if any(Path(str(path) + suffix).exists() for suffix in ("-wal", "-journal")):
        raise ValueError(f"Refusing immutable read with a SQLite transaction sidecar: {path}")
    conn = sqlite3.connect(path.resolve().as_uri() + "?mode=ro&immutable=1", uri=True)
    conn.execute("PRAGMA query_only=ON")
    return conn


class Cdr1:
    def __init__(self, blob):
        byte_order = {b"\x00\x01": "<", b"\x00\x00": ">"}.get(blob[:2])
        if byte_order is None or blob[2:4] != b"\x00\x00":
            raise ValueError(f"Unsupported CDR encapsulation: {blob[:4].hex()}")
        self.blob, self.order, self.pos = memoryview(blob), byte_order, 4

    def take(self, length):
        end = self.pos + length
        if end > len(self.blob):
            raise ValueError("Truncated CDR payload")
        data = self.blob[self.pos:end]
        self.pos = end
        return data

    def scalar(self, code):
        size = struct.calcsize(code)
        self.take(-(self.pos - 4) % size)
        return struct.unpack(self.order + code, self.take(size))[0]

    def boolean(self):
        value = self.scalar("B")
        if value not in (0, 1):
            raise ValueError(f"Invalid CDR boolean: {value}")
        return bool(value)

    def string(self):
        data = bytes(self.take(self.scalar("I")))
        if not data or data[-1] != 0 or b"\x00" in data[:-1]:
            raise ValueError("Invalid CDR string")
        return data[:-1].decode("utf-8")


def decode(blob):
    reader = Cdr1(blob)
    sec, nsec = reader.scalar("i"), reader.scalar("I")
    if nsec >= 10**9:
        raise ValueError("Invalid Header nanosecond component")
    frame = reader.string()
    height, width, count = (reader.scalar("I") for _ in range(3))
    if count > 1024:
        raise ValueError(f"Unsupported field count: {count}")
    layout = [(reader.string(), reader.scalar("I"), reader.scalar("B"), reader.scalar("I"))
              for _ in range(count)]
    bigendian = reader.boolean()
    point_step, row_step, size = (reader.scalar("I") for _ in range(3))
    if height != 1 or not point_step or row_step < width * point_step or size != height * row_step:
        raise ValueError(f"Unsupported or invalid dimensions: {height=}, {width=}, {point_step=}, {row_step=}, {size=}")
    payload = reader.take(size)
    dense = reader.boolean()
    if reader.pos != len(blob):
        raise ValueError("Unexpected trailing CDR data")
    names = [name for name, *_ in layout]
    if len(set(names)) != len(names) or set(names) != set(REQUIRED):
        raise ValueError(f"Unsupported PointCloud2 fields: {layout}")
    order = ">" if bigendian else "<"
    columns = {}
    occupied = set()
    for name, offset, datatype, items in layout:
        if datatype != REQUIRED[name] or items != 1:
            raise ValueError(f"Unsupported field datatype/count: {name}, {datatype}, {items}")
        dtype = np.dtype(order + TYPES[datatype])
        span = set(range(offset, offset + dtype.itemsize))
        if offset + dtype.itemsize > point_step or occupied & span:
            raise ValueError(f"Invalid/overlapping offset: {name} at {offset}")
        occupied.update(span)
        # 逐字段按实际字节步长取值。时间戳独立保留 FLOAT64。
        columns[name] = np.ndarray((width,), dtype, payload, offset, (point_step,)) if width else np.empty(0, dtype)
    points = np.column_stack([columns[name] for name in FIELDS])
    return {
        "points": points, "fields": np.asarray(FIELDS), "tag": columns["tag"], "line": columns["line"],
        "timestamp": columns["timestamp"], "header_stamp_ns": np.int64(sec * 10**9 + nsec),
        "header_sec": np.int32(sec), "header_nanosec": np.uint32(nsec), "frame_id": frame,
        "width": width, "height": height, "point_step": point_step, "row_step": row_step,
        "point_is_bigendian": bigendian, "is_dense": dense, "pointfield_layout_json": json.dumps(layout),
    }


def verify_kit_layout(blob, arrays):
    """Independent check for the observed kit layout using packed records, from BLOB tail."""
    expected = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("intensity", 12, 7, 1),
                ("tag", 16, 2, 1), ("line", 17, 2, 1), ("timestamp", 18, 8, 1)]
    if (sorted(map(tuple, json.loads(str(arrays["pointfield_layout_json"]))), key=lambda f: f[1]) != expected
            or arrays["point_step"] != 26 or arrays["row_step"] != arrays["width"] * 26
            or arrays["point_is_bigendian"] or blob[:4] != b"\x00\x01\x00\x00"):
        raise ValueError("Observed field-kit layout differs, cannot independently verify with packed-record reference")
    count = int(arrays["width"])
    start = len(blob) - 1 - 26 * count
    assert start >= 4 and struct.unpack_from("<I", blob, start - 4)[0] == count * 26
    packed = list(struct.iter_unpack("<ffffBBQ", memoryview(blob)[start:-1]))
    points = np.array([row[:4] for row in packed], dtype=np.float32).reshape(count, 4)
    # 比较数值及符号位。FLOAT64 时间使用原始 64 位字，不通过 Python float。
    assert np.array_equal(points, arrays["points"], equal_nan=True)
    finite = ~np.isnan(points)
    assert np.array_equal(points.view("u4")[finite], arrays["points"].view("u4")[finite])
    for key, index in (("tag", 4), ("line", 5), ("timestamp", 6)):
        actual = arrays[key].view("<u8") if key == "timestamp" else arrays[key]
        assert np.array_equal(actual, [row[index] for row in packed] if key != "timestamp" else
                              np.array([row[index] for row in packed], dtype="uint64"))
    sec, nsec, length = struct.unpack_from("<iII", blob, 4)
    assert sec * 10**9 + nsec == arrays["header_stamp_ns"]
    assert blob[16:16 + length] == str(arrays["frame_id"]).encode() + b"\x00"
    dims_offset = (16 + length + 3) & ~3
    assert struct.unpack_from("<II", blob, dims_offset) == (1, count)
    assert np.count_nonzero((points[:, :3] == 0).all(1)) == np.count_nonzero((arrays["points"][:, :3] == 0).all(1))
    assert np.count_nonzero(~np.isfinite(points).all(1)) == np.count_nonzero(~np.isfinite(arrays["points"]).all(1))


def verify_file(path, arrays, bag_stamp, blob=None):
    with np.load(path, allow_pickle=False) as saved:
        if set(saved.files) != set(arrays):
            raise ValueError(f"Existing NPZ has unexpected fields: {path}")
        for key, value in arrays.items():
            actual, expected = saved[key], np.asarray(value)
            if actual.shape != expected.shape or actual.dtype != expected.dtype or actual.tobytes() != expected.tobytes():
                raise ValueError(f"Existing or exported NPZ differs in {key}: {path}")
        assert int(saved["bag_stamp_ns"]) == bag_stamp
    xyz, intensity, fields = read_cloud_data(path, verbose=False)
    assert list(FIELDS) == fields
    assert xyz.tobytes() == arrays["points"][:, :3].tobytes()
    assert intensity.tobytes() == arrays["points"][:, 3].tobytes()
    if blob is not None:
        with np.load(path, allow_pickle=False) as saved:
            verify_kit_layout(blob, saved)


def write_new_or_identical(path, data):
    try:
        with path.open("xb") as file:
            file.write(data)
    except FileExistsError:
        if path.read_bytes() != data:
            raise ValueError(f"Refusing to overwrite a different file: {path}")


def convert(data_root, output, representative_only):
    output.mkdir(parents=True, exist_ok=True)
    result = {"phase": "representative" if representative_only else "all", "bags": {}, "failures": []}
    for bag, (expected_count, representatives) in BAGS.items():
        db = data_root / bag / f"{bag}_0.db3"
        folder = output / bag
        folder.mkdir(exist_ok=True)
        entries = result["bags"][bag] = {"source": str(db.resolve()), "messages": []}
        with closing(open_bag(db)) as con:
            topics = con.execute("SELECT id,type,serialization_format FROM topics WHERE name=?", (TOPIC,)).fetchall()
            if len(topics) != 1 or topics[0][1:] != ("sensor_msgs/msg/PointCloud2", "cdr"):
                raise ValueError(f"{bag}: unsupported topic type/serialization {topics}")
            tid = topics[0][0]
            count = con.execute("SELECT count(*) FROM messages WHERE topic_id=?", (tid,)).fetchone()[0]
            if count != expected_count:
                raise ValueError(f"{bag}: expected {expected_count}, found {count} messages")
            sql = "SELECT id,timestamp,data FROM messages WHERE topic_id=?"
            args = (tid,)
            if representative_only:
                sql += " AND id IN (?,?,?)"
                args += representatives
            seen = set()
            for mid, stamp, blob in con.execute(sql + " ORDER BY timestamp,id", args):
                seen.add(mid)
                try:
                    arrays = decode(blob)
                    arrays.update(bag_stamp_ns=np.int64(stamp), source_message_id=np.int64(mid), source_bag=bag,
                                  source_db3=str(db.resolve()), source_topic=TOPIC,
                                  source_message_sha256=hashlib.sha256(blob).hexdigest())
                    path = folder / f"message_{mid:06d}.npz"
                    verify_kit_layout(blob, arrays)
                    if not path.exists():
                        data = BytesIO()
                        np.savez_compressed(data, **arrays)
                        write_new_or_identical(path, data.getvalue())
                    # 所有消息校验读回，额外用独立的逐点解析交叉核对。
                    verify_file(path, arrays, stamp, blob)
                    points = arrays["points"]
                    timestamps = arrays["timestamp"]
                    entries["messages"].append({
                        "id": mid, "path": str(path.relative_to(output)), "sha256": arrays["source_message_sha256"],
                        "points": len(points), "zero_xyz": int(np.count_nonzero((points[:, :3] == 0).all(1))),
                        "nonfinite_rows": int(np.count_nonzero(~np.isfinite(points).all(1))),
                        "bag_stamp_ns": stamp, "header_stamp_ns": int(arrays["header_stamp_ns"]),
                        "point_timestamp_min": float(timestamps.min()) if len(timestamps) else None,
                        "point_timestamp_max": float(timestamps.max()) if len(timestamps) else None,
                        "frame_id": arrays["frame_id"],
                    })
                except Exception as error:
                    result["failures"].append({"bag": bag, "id": mid,
                                               "reason": f"{type(error).__name__}: {error or 'validation assertion'}"})
                    if representative_only:
                        break
            if representative_only and seen != set(representatives):
                result["failures"].append({"bag": bag, "reason": "Representative verification incomplete",
                                           "missing_ids": sorted(set(representatives) - seen)})
        messages = entries["messages"]
        entries.update(success_count=len(messages), expected_count=len(representatives) if representative_only else count)
        for key in ("points", "zero_xyz", "nonfinite_rows"):
            entries[key] = sum(row[key] for row in messages)
        for key in ("bag_stamp_ns", "header_stamp_ns", "point_timestamp_min", "point_timestamp_max"):
            values = [row[key] for row in messages if row[key] is not None]
            entries[key + "_range"] = [min(values), max(values)] if values else None
        if representative_only and result["failures"]:
            break
    result["success_count"] = sum(item["success_count"] for item in result["bags"].values())
    result["failure_count"] = len(result["failures"])
    name = "representative_validation.json" if representative_only else "conversion_summary.json"
    if result["failures"]:
        # 失败报告独立保存，便于重试且不覆盖成功清单。
        name = name.removesuffix(".json") + "_failed_" + hashlib.sha256(
            json.dumps(result["failures"], sort_keys=True).encode()).hexdigest()[:12] + ".json"
    write_new_or_identical(output / name, (json.dumps(result, indent=2, allow_nan=False) + "\n").encode())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--representative-only", action="store_true")
    args = parser.parse_args()
    if not args.representative_only:
        verified = json.loads((args.output / "representative_validation.json").read_text(encoding="utf-8"))
        if verified["success_count"] != 9 or verified["failures"]:
            raise SystemExit("Run the nine-message representative validation before batch conversion")
    result = convert(args.data_root, args.output, args.representative_only)
    print(json.dumps({k: result[k] for k in ("phase", "success_count", "failure_count", "failures")}))
    return int(bool(result["failures"]))


if __name__ == "__main__":
    raise SystemExit(main())
