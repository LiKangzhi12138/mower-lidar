"""R3.8: check output modes against one shared computation and original NPZ."""
import json
import shutil
from pathlib import Path
import numpy as np
from PIL import Image
from offline_detector import run, analysis_coordinates
import open3d as o3d  # 在共用算法模块设置 OpenMP 线程后再导入。
from radar_view import export_result, export_memory
from run_perception import run_pipeline, DEFAULT_CONFIG
from npz_to_ply import read_cloud_data
from check_r3_radar_view import check_geometry

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "outputs/r3_performance/correctness"
DATA = Path(r"E:\workspace\mower-data\data\newdata")


def main():
    check_geometry()
    rows=[]
    for number in range(142,150):
        source=DATA/f"sample_{number:06d}"/"lidar/cloud_01.npz"
        folder=OUT/f"sample_{number:06d}"; context={}
        result, detection=run(source,DEFAULT_CONFIG,folder,"detection",context=context)
        shutil.copyfile(detection/"clusters_colored.ply",detection/"obstacles_colored.ply")
        full=export_result(detection/"result.json",folder/"full")
        core=export_memory(result,context,folder/"core")
        # 同一份拟合与标签分别输出，避免把随机平面波动误当作模式差异。
        for a,b in zip(full["candidates"],core["candidates"]):
            assert {k:v for k,v in a.items() if k!="point_cloud"} == {k:v for k,v in b.items() if k!="point_cloud"}
        assert full["candidate_count"]==core["candidate_count"]==len(core["candidates"])
        raw,_,_=read_cloud_data(source,verbose=False)
        for item in core["candidates"]:
            sensor=raw[item["input_indices"]].astype(np.float64)
            xyz,frame=analysis_coordinates(sensor,result["parameters"]["coordinates"])
            assert frame==item["coordinate_frame"]
            assert np.allclose(xyz.mean(0),item["center_xyz"])
            assert np.allclose(xyz.min(0),item["bbox_min_xyz"])
            assert np.allclose(xyz.max(0),item["bbox_max_xyz"])
            assert np.allclose(np.ptp(xyz,axis=0),item["bbox_extent_xyz"])
            assert np.isclose(np.linalg.norm(sensor,axis=1).min(),item["nearest_sensor_distance"])
            assert np.isclose(np.hypot(xyz[:,0],xyz[:,1]).min(),item["nearest_range_xy"])
            assert item["point_cloud"] is None
        for key in ("selected_plane_diagnostics","rejected_plane_candidates","removed_point_count"):
            assert core[key]==result[key]
        assert [p.name for p in (folder/"core").iterdir()]==["obstacles.json"]
        cloud=o3d.io.read_point_cloud(str(folder/"full/obstacles_colored.ply"))
        assert len(cloud.points)==len(cloud.colors)==result["candidate_region_point_count"]
        assert np.allclose(np.asarray(cloud.points),context["analysis_xyz"][context["candidate_rows"]])
        with Image.open(folder/"full/radar_view.png") as picture: picture.verify()
        historical=json.loads((ROOT/f"outputs/r3_radar_view/sample_{number:06d}/obstacles.json").read_text(encoding="utf-8"))
        assert full["candidates"]==historical["candidates"]
        rows.append({"sample":number,"candidates":result["candidate_count"],"status":"passed"})
    for name, points in (("empty",[]),("zero",[[0,0,0,1]]),("sparse",[[1,0,0,1],[5,0,0,1]])):
        path=OUT/(name+".npz"); np.savez_compressed(path,points=np.asarray(points,dtype=float).reshape(-1,4),fields=np.array(["x","y","z","intensity"]))
        result,doc=run_pipeline(path,OUT/(name+"_core"),mode="core")
        assert doc["candidate_count"]==0
        assert doc["removed_point_count"]==0
    report={"status":"passed","shared_result_samples":rows,"edge_cases":["empty","zero","sparse"],"four_directions_origin":"passed"}
    (OUT/"checks.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report))


if __name__=="__main__": main()
