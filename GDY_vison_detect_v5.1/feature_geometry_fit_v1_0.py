"""对FeatureROIProcessor输出进行安装面、螺栓中心和FLG高度计算。"""

VERSION="1.0.0"

import argparse
import json
from pathlib import Path
from typing import Dict,Optional,Union

import numpy as np


class FeatureGeometryFitter:
    """纯计算接口，不修改输入ROI结果。坐标和距离单位均为mm。"""

    @staticmethod
    def _roi_points(roi:dict)->np.ndarray:
        points=np.asarray(roi["cut_points"],dtype=np.float64)
        mask=np.asarray(roi["pixel_mask"],dtype=bool)
        if points.ndim!=3 or points.shape[2]!=3:
            raise ValueError(f"cut_points必须为h×w×3，当前形状: {points.shape}")
        if mask.shape!=points.shape[:2]:
            raise ValueError("pixel_mask与cut_points尺寸不一致")
        valid=mask&np.isfinite(points).all(axis=2)
        return points[valid]

    @staticmethod
    def _orient_normal_to_camera(normal:np.ndarray,center:np.ndarray)->np.ndarray:
        """点云位于相机坐标系时，使法向从安装面指向相机光心。"""
        toward_camera=-np.asarray(center,dtype=np.float64)
        if float(np.dot(normal,toward_camera))<0:
            normal=-normal
        return normal

    @classmethod
    def fit_plane_ransac(
        cls,
        points:np.ndarray,
        distance_threshold_mm:float=1.5,
        max_iterations:int=1000,
        min_inlier_ratio:float=0.6,
        random_seed:int=0,
    )->dict:
        """RANSAC拟合平面，最终使用全部内点SVD精拟合。"""
        points=np.asarray(points,dtype=np.float64)
        points=points[np.isfinite(points).all(axis=1)]
        if len(points)<3:
            raise ValueError("安装面有效点少于3个")
        if distance_threshold_mm<=0 or max_iterations<=0:
            raise ValueError("平面拟合阈值和迭代次数必须大于0")
        rng=np.random.default_rng(random_seed)
        best_mask=None
        best_count=0
        best_error=np.inf
        for _ in range(int(max_iterations)):
            sample=points[rng.choice(len(points),3,replace=False)]
            normal=np.cross(sample[1]-sample[0],sample[2]-sample[0])
            length=float(np.linalg.norm(normal))
            if length<1e-9:
                continue
            normal/=length
            d=-float(np.dot(normal,sample[0]))
            distances=np.abs(points@normal+d)
            mask=distances<=distance_threshold_mm
            count=int(np.count_nonzero(mask))
            if count<3:
                continue
            error=float(np.mean(distances[mask]))
            if count>best_count or (count==best_count and error<best_error):
                best_mask=mask
                best_count=count
                best_error=error
        if best_mask is None:
            raise RuntimeError("RANSAC未找到有效安装平面")
        inlier_ratio=best_count/len(points)
        if inlier_ratio<min_inlier_ratio:
            raise ValueError(f"安装面内点率过低: {inlier_ratio:.3f}<{min_inlier_ratio:.3f}")
        inliers=points[best_mask]
        center=np.mean(inliers,axis=0)
        _,_,vh=np.linalg.svd(inliers-center,full_matrices=False)
        normal=vh[-1]
        normal/=np.linalg.norm(normal)
        normal=cls._orient_normal_to_camera(normal,center)
        d=-float(np.dot(normal,center))
        signed=points@normal+d
        refined_mask=np.abs(signed)<=distance_threshold_mm
        refined_points=points[refined_mask]
        if len(refined_points)>=3:
            center=np.mean(refined_points,axis=0)
            _,_,vh=np.linalg.svd(refined_points-center,full_matrices=False)
            normal=vh[-1]
            normal/=np.linalg.norm(normal)
            normal=cls._orient_normal_to_camera(normal,center)
            d=-float(np.dot(normal,center))
        else:
            refined_mask=best_mask
            refined_points=inliers
        residuals=refined_points@normal+d
        rms=float(np.sqrt(np.mean(residuals**2)))
        return {
            "center_camera_mm":center,
            "normal_camera":normal,
            "entry_direction_camera":-normal,
            "d":d,
            "rms_mm":rms,
            "inlier_count":int(len(refined_points)),
            "source_point_count":int(len(points)),
            "inlier_ratio":float(len(refined_points)/len(points)),
            "distance_threshold_mm":float(distance_threshold_mm),
        }

    @staticmethod
    def fit_bolt_center(
        roi:dict,
        center_local_px:np.ndarray,
        label_radius_px:float,
        patch_radius_px:float=5.0,
        min_valid_points:int=5,
    )->dict:
        """取标签圆心周围小圆域内的有效三维点均值作为螺栓中心。"""
        points=np.asarray(roi["cut_points"],dtype=np.float64)
        roi_mask=np.asarray(roi["pixel_mask"],dtype=bool)
        center=np.asarray(center_local_px,dtype=np.float64)
        if points.ndim!=3 or points.shape[2]!=3 or roi_mask.shape!=points.shape[:2]:
            raise ValueError("螺栓ROI格式错误")
        radius=min(float(patch_radius_px),float(label_radius_px))
        if radius<=0:
            raise ValueError("bolt_patch_radius_px必须大于0")
        yy,xx=np.ogrid[:points.shape[0],:points.shape[1]]
        patch=(xx-center[0])**2+(yy-center[1])**2<=radius**2
        valid=patch&roi_mask&np.isfinite(points).all(axis=2)
        samples=points[valid]
        if len(samples)<min_valid_points:
            raise ValueError(f"螺栓中心小区域有效点不足: {len(samples)}<{min_valid_points}")
        mean=np.mean(samples,axis=0)
        std=np.std(samples,axis=0)
        return {
            "center_camera_mm":mean,
            "center_local_px":center,
            "patch_radius_px":radius,
            "valid_point_count":int(len(samples)),
            "std_xyz_mm":std,
        }

    @classmethod
    def fit_flg(
        cls,
        roi:dict,
        plane_center:np.ndarray,
        surface_normal:np.ndarray,
        height_mm:float=35.0,
    )->dict:
        """将FLG点云均值投影到安装面，并沿外法向增加给定高度。"""
        if height_mm<=0:
            raise ValueError("flg_height_mm必须大于0")
        points=cls._roi_points(roi)
        if len(points)<3:
            raise ValueError("FLG有效点少于3个")
        mean=np.mean(points,axis=0)
        normal=np.asarray(surface_normal,dtype=np.float64)
        normal/=np.linalg.norm(normal)
        plane_center=np.asarray(plane_center,dtype=np.float64)
        signed_distance=float(np.dot(mean-plane_center,normal))
        base_center=mean-signed_distance*normal
        top_center=base_center+float(height_mm)*normal
        return {
            "point_mean_camera_mm":mean,
            "base_center_camera_mm":base_center,
            "top_center_camera_mm":top_center,
            "height_mm":float(height_mm),
            "height_direction_camera":normal,
            "valid_point_count":int(len(points)),
        }

    @staticmethod
    def build_feature_frame(
        ls1_center:np.ndarray,
        ls2_center:np.ndarray,
        surface_normal:np.ndarray,
    )->dict:
        """以两螺栓中点为原点，LS1到LS2为X轴，安装面外法向为Z轴。"""
        p1=np.asarray(ls1_center,dtype=np.float64)
        p2=np.asarray(ls2_center,dtype=np.float64)
        z=np.asarray(surface_normal,dtype=np.float64)
        z/=np.linalg.norm(z)
        delta=p2-p1
        x=delta-float(np.dot(delta,z))*z
        length=float(np.linalg.norm(x))
        if length<1e-9:
            raise ValueError("两个螺栓中心连线无法确定特征坐标系X轴")
        x/=length
        y=np.cross(z,x)
        y/=np.linalg.norm(y)
        x=np.cross(y,z)
        x/=np.linalg.norm(x)
        origin=(p1+p2)/2.0
        rotation=np.column_stack((x,y,z))
        transform=np.eye(4,dtype=np.float64)
        transform[:3,:3]=rotation
        transform[:3,3]=origin
        return {
            "origin_camera_mm":origin,
            "x_axis_camera":x,
            "y_axis_camera":y,
            "z_axis_camera":z,
            "rotation_camera_feature":rotation,
            "T_camera_feature":transform,
            "bolt_distance_mm":float(np.linalg.norm(p2-p1)),
        }

    @classmethod
    def fit(
        cls,
        feature_roi_result:dict,
        flg_height_mm:float=35.0,
        bolt_patch_radius_px:float=5.0,
        min_bolt_points:int=5,
        plane_distance_threshold_mm:float=1.5,
        plane_max_iterations:int=1000,
        plane_min_inlier_ratio:float=0.6,
        random_seed:int=0,
    )->dict:
        """统一拟合入口，输入为FeatureROIProcessor.process()的完整结果。"""
        rois=feature_roi_result.get("rois",{})
        features=feature_roi_result.get("features",{})
        missing=[name for name in ("LS1","LS2","FLG","PLANE") if name not in rois]
        if missing:
            raise KeyError(f"缺少ROI结果: {missing}")
        plane_points=cls._roi_points(rois["PLANE"])
        plane=cls.fit_plane_ransac(
            plane_points,
            distance_threshold_mm=plane_distance_threshold_mm,
            max_iterations=plane_max_iterations,
            min_inlier_ratio=plane_min_inlier_ratio,
            random_seed=random_seed,
        )
        bolts={}
        for label in ("LS1","LS2"):
            info=features[label]
            bolts[label]=cls.fit_bolt_center(
                rois[label],
                info["center_local_px"],
                info["radius_px"],
                patch_radius_px=bolt_patch_radius_px,
                min_valid_points=min_bolt_points,
            )
        flg=cls.fit_flg(
            rois["FLG"],
            plane["center_camera_mm"],
            plane["normal_camera"],
            height_mm=flg_height_mm,
        )
        frame=cls.build_feature_frame(
            bolts["LS1"]["center_camera_mm"],
            bolts["LS2"]["center_camera_mm"],
            plane["normal_camera"],
        )
        return {
            "version":VERSION,
            "prefix":str(feature_roi_result.get("prefix","capture")),
            "coordinate_frame":"camera",
            "unit":"mm",
            "plane":plane,
            "bolts":bolts,
            "flg":flg,
            "feature_frame":frame,
        }

    @staticmethod
    def _json_value(value):
        if isinstance(value,np.ndarray):
            return value.tolist()
        if isinstance(value,np.generic):
            return value.item()
        if isinstance(value,dict):
            return {key:FeatureGeometryFitter._json_value(item) for key,item in value.items()}
        if isinstance(value,(list,tuple)):
            return [FeatureGeometryFitter._json_value(item) for item in value]
        return value

    @classmethod
    def save_json(cls,result:dict,path:Union[str,Path])->Path:
        """保存接口；纯拟合请只调用fit()。"""
        path=Path(path)
        path.parent.mkdir(parents=True,exist_ok=True)
        with open(path,"w",encoding="utf-8") as file:
            json.dump(cls._json_value(result),file,ensure_ascii=False,indent=2)
        return path


def process_capture_geometry(
    data_dir:Union[str,Path],
    prefix:str,
    flg_height_mm:float=35.0,
    bolt_patch_radius_px:float=5.0,
    save_roi_ply:bool=False,
    save_fit_json:bool=True,
    roi_kwargs:Optional[dict]=None,
    **fit_kwargs,
)->dict:
    """主线程兼容入口：先执行ROI提取，再执行本文件拟合。

    roi_kwargs 透传给 feature_roi_main_v1_0.process_capture 的 ROI 阶段参数
    （plane_clearance_px / plane_half_width_px / exclusion_margin_px /
     low_percentile / high_percentile / margin_mm / min_plane_points 等）。
    """
    from feature_roi_main_v1_0 import process_capture
    roi_result=process_capture(
        data_dir,prefix,
        save_output=save_roi_ply,
        **(roi_kwargs or {}),
    )
    fit_result=FeatureGeometryFitter.fit(
        roi_result,
        flg_height_mm=flg_height_mm,
        bolt_patch_radius_px=bolt_patch_radius_px,
        **fit_kwargs,
    )
    if save_fit_json:
        output=Path(data_dir)/f"{prefix}_geometry_fit_v1_0.json"
        FeatureGeometryFitter.save_json(fit_result,output)
        fit_result["saved_json"]=str(output)
    return fit_result


def _print_summary(result:dict)->None:
    plane=result["plane"]
    print(f"VERSION={result['version']}, prefix={result['prefix']}")
    print(f"plane_normal={plane['normal_camera']}")
    print(f"entry_direction={plane['entry_direction_camera']}")
    print(f"plane_rms={plane['rms_mm']:.4f}mm, inlier_ratio={plane['inlier_ratio']:.4f}")
    for label,info in result["bolts"].items():
        print(f"{label}_center={info['center_camera_mm']}, points={info['valid_point_count']}")
    print(f"FLG_base={result['flg']['base_center_camera_mm']}")
    print(f"FLG_top={result['flg']['top_center_camera_mm']}, height={result['flg']['height_mm']:.2f}mm")
    print(f"bolt_distance={result['feature_frame']['bolt_distance_mm']:.3f}mm")
    if "saved_json" in result:
        print(f"保存结果: {result['saved_json']}")


def main()->None:
    parser=argparse.ArgumentParser(description="拟合安装面、螺栓中心和FLG高度")
    parser.add_argument("data_dir",help="数据目录")
    parser.add_argument("--prefix",required=True,help="数据前缀，例如1_1")
    parser.add_argument("--flg-height",type=float,default=35.0,help="FLG相对安装面高度，mm")
    parser.add_argument("--bolt-patch-radius",type=float,default=5.0,help="螺栓中心取均值小圆半径，像素")
    parser.add_argument("--plane-threshold",type=float,default=1.5,help="平面RANSAC距离阈值，mm")
    parser.add_argument("--save-roi-ply",action="store_true",help="同时保存四个ROI PLY")
    parser.add_argument("--no-save-json",action="store_true",help="不保存拟合JSON")
    args=parser.parse_args()
    result=process_capture_geometry(
        args.data_dir,args.prefix,
        flg_height_mm=args.flg_height,
        bolt_patch_radius_px=args.bolt_patch_radius,
        plane_distance_threshold_mm=args.plane_threshold,
        save_roi_ply=args.save_roi_ply,
        save_fit_json=not args.no_save_json,
    )
    _print_summary(result)


if __name__=="__main__":
    main()
