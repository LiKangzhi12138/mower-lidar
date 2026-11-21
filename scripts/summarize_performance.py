"""Summarize saved R3.8 measurements and verify artifact/numeric compatibility."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent / "outputs/r3_performance"
FOLDERS = {"baseline": "baseline_final2", "full": "full_final", "core": "core_final"}


def stats(values, scale=1):
    return dict(zip(("p50", "p95", "max"), map(float, np.percentile(np.asarray(values)/scale,[50,95,100]))))


def main():
    summary={}; signatures={}
    for mode,folder in FOLDERS.items():
        data=json.loads((ROOT/folder/"measurements.json").read_text())
        warm,cold=data["warm"],data["cold"]; assert len(warm)==160
        keys=set().union(*(r["stages_ns"] for r in warm))
        summary[mode]={"runs":len(warm),"wall_ms":stats([r["wall_ns"] for r in warm],1e6),
            "cpu_ms":stats([r["cpu_ns"] for r in warm],1e6),
            "sampled_peak_working_set_mib":stats([r["sampled_peak_working_set_bytes"] for r in warm],2**20),
            "stages_ms":{k:stats([r["stages_ns"].get(k,0) for r in warm],1e6) for k in sorted(keys)},
            "cold_process_ms":stats([r["process_wall_ns"] for r in cold],1e6),
            "cold_pipeline_ms":stats([r["wall_ns"] for r in cold],1e6),"per_sample":{}}
        signatures[mode]={}
        for n in range(142,150):
            name=f"sample_{n:06d}"; rows=[r for r in warm if r["sample"]==name]; assert len(rows)==20
            summary[mode]["per_sample"][name]={"wall_ms":stats([r["wall_ns"] for r in rows],1e6),
                "files":rows[0]["files"],"bytes":rows[0]["bytes"],"calls":rows[0]["calls"],
                "points":{k:rows[0][k] for k in ("raw_points","clean_points","nonground_points","candidates","noise_points")}}
            hashes=set()
            for r in rows:
                doc=json.loads((ROOT/folder/f"run_{n}_{r['repeat']:02d}"/"obstacles.json").read_text(encoding="utf8"))
                normalized=[{k:v for k,v in c.items() if k!="point_cloud"} for c in doc["candidates"]]
                hashes.add(hashlib.sha256(json.dumps(normalized,sort_keys=True).encode()).hexdigest())
            assert len(hashes)==1,(mode,n,"RANSAC variation")
            signatures[mode][name]=next(iter(hashes))
    assert signatures["baseline"]==signatures["full"]==signatures["core"]
    matches=0
    for n in range(142,150):
        a=ROOT/FOLDERS["baseline"]/f"run_{n}_00";b=ROOT/FOLDERS["full"]/f"run_{n}_00"
        assert {str(p.relative_to(a)) for p in a.rglob("*") if p.is_file()}=={str(p.relative_to(b)) for p in b.rglob("*") if p.is_file()}
        for p in a.rglob("*"):
            if p.suffix in (".ply",".png"):
                assert p.read_bytes()==(b/p.relative_to(a)).read_bytes(),str(p)
                matches+=1
    meta=json.loads((ROOT/"baseline_v2/metadata.json").read_text())
    for name in ("run_perception.py","offline_detector.py","radar_view.py"):
        assert meta["source_hashes"][str(Path("scripts")/name)]==hashlib.sha256((ROOT/"original_source"/name).read_bytes()).hexdigest()
    summary["validation"]={"normalized_candidates_all_480_runs_equal":True,"no_observed_ransac_variation":True,
        "matching_full_baseline_ply_png_first_run_all_frames":matches,"snapshot_matches_prechange_baseline":True}
    (ROOT/"summary.json").write_text(json.dumps(summary,indent=2),encoding="utf8")
    for mode in FOLDERS:
        print(mode, {k:summary[mode][k] for k in ("wall_ms","cpu_ms","sampled_peak_working_set_mib","cold_process_ms")})
        print("stages_p50_ms",{k:round(v["p50"],4) for k,v in summary[mode]["stages_ms"].items()})
    print(summary["validation"])


if __name__=="__main__": main()
