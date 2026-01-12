#!/usr/bin/env python3
"""
screen_detection_data_collector.py
================================

基于视觉识别的人形机器人精准按触技术 - 数据采集脚本

功能：
1. 使用 RealSense 摄像头采集触摸屏图像数据
2. 交互式标注屏幕区域和网格分割
3. 保存标注数据用于后续模型训练
4. 同时保存彩色图像和深度图像
5. 支持多种分辨率采集：1920x1080, 960x540, 640x480
6. 添加屏幕区域边界框标注用于目标检测模型训练

依赖：
- unitree_sdk2py.camera.realsense_camera_client (RealSense 接口)
- opencv-python, numpy
"""

import os
import sys
import json
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any

import cv2
import numpy as np

# 导入封装好的 RealSense 摄像头类
try:
    from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera
except ImportError:
    # 兼容本地开发环境
    current_dir = Path(__file__).parent.parent / "loco" / "unitree_sdk_python" / "unitree_sdk2py" / "camera"
    sys.path.insert(0, str(current_dir))
    from realsense_camera_client import RealSenseCamera


# 支持的分辨率配置
SUPPORTED_RESOLUTIONS = {
    "1920x1080": (1920, 1080),
    "1280x720": (1280, 720),
    "960x540": (960, 540),
    "640x480": (640, 480),
    "high": (1920, 1080),     # 别名
    "medium": (1280, 720),    # 别名
    "low": (640, 480),        # 别名
}


class ScreenDetectionDataCollector:
    """屏幕检测数据采集器 - 支持多分辨率和目标检测标注"""
    
    def __init__(self, output_dir: str = "data/screen_detection", width: int = 1920, height: int = 1080):
        """
        初始化数据采集器
        
        Args:
            output_dir: 数据保存根目录
            width: 图像宽度
            height: 图像高度
        """
        self.width = width
        self.height = height
        self.resolution_name = f"{width}x{height}"
        
        # 根据分辨率创建特定的输出目录
        self.output_dir = Path(output_dir) / self.resolution_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 创建子目录
        (self.output_dir / "images").mkdir(exist_ok=True)
        (self.output_dir / "depth").mkdir(exist_ok=True)
        (self.output_dir / "depth_colored").mkdir(exist_ok=True)
        (self.output_dir / "annotations").mkdir(exist_ok=True)
        (self.output_dir / "grids").mkdir(exist_ok=True)
        (self.output_dir / "yolo_labels").mkdir(exist_ok=True)
        (self.output_dir / "coco_labels").mkdir(exist_ok=True)
        
        # 创建分辨率配置文件
        self._save_resolution_config()
        
        # 标注状态
        self.current_color_image = None
        self.current_depth_raw = None
        self.current_depth_colored = None
        self.screen_corners = []
        self.grid_points = []
        self.is_annotating = False
        self.current_sample_id = 0
        
        # 显示参数（根据分辨率自适应）
        self.display_scale = self._get_display_scale()
        self.point_radius = max(3, int(5 * min(width/1920, height/1080)))
        self.line_thickness = max(1, int(2 * min(width/1920, height/1080)))
        
        # 网格参数
        self.grid_rows = 6
        self.grid_cols = 6
        
        # 使用封装好的 RealSense 摄像头
        self.camera = RealSenseCamera(width=width, height=height, fps=30)
        
        print(f"✅ 数据采集器初始化完成")
        print(f"   分辨率: {self.resolution_name}")
        print(f"   输出目录: {self.output_dir}")
        print(f"   显示缩放: {self.display_scale:.2f}")
    
    def _get_display_scale(self) -> float:
        """根据分辨率自动计算显示缩放比例"""
        max_display_width = 1200
        max_display_height = 900
        
        scale_x = max_display_width / self.width
        scale_y = max_display_height / self.height
        scale = min(scale_x, scale_y, 1.0)
        
        return scale
    
    def _save_resolution_config(self):
        """保存分辨率配置信息"""
        config_data = {
            "resolution": self.resolution_name,
            "width": self.width,
            "height": self.height,
            "created_time": datetime.now().isoformat(),
            "purpose": "Screen detection data collection with bounding box annotations",
            "grid_size": [6, 6],
            "class_names": ["screen"],
            "annotation_formats": ["yolo", "coco", "custom"],
        }
        
        config_path = self.output_dir / "resolution_config.json"
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(config_data, f, indent=2, ensure_ascii=False)
    
    def _calculate_screen_bbox(self) -> Dict[str, Any]:
        """基于四个角点计算屏幕区域的边界框"""
        if len(self.screen_corners) != 4:
            return {}
        
        x_coords = [p[0] for p in self.screen_corners]
        y_coords = [p[1] for p in self.screen_corners]
        
        x_min = min(x_coords)
        y_min = min(y_coords)
        x_max = max(x_coords)
        y_max = max(y_coords)
        
        width = x_max - x_min
        height = y_max - y_min
        
        center_x = x_min + width / 2
        center_y = y_min + height / 2
        
        return {
            "coco_bbox": [x_min, y_min, width, height],
            "yolo_bbox": [
                center_x / self.width,
                center_y / self.height,
                width / self.width,
                height / self.height
            ],
            "xyxy_bbox": [x_min, y_min, x_max, y_max],
            "xywh_bbox": [center_x, center_y, width, height],
            "polygon_corners": self.screen_corners,
            "area": width * height,
            "aspect_ratio": width / height if height > 0 else 0
        }
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数，用于标注屏幕角点"""
        if not self.is_annotating or self.current_color_image is None:
            return
        
        if event == cv2.EVENT_LBUTTONDOWN:
            orig_x = int(x / self.display_scale)
            orig_y = int(y / self.display_scale)
            
            if len(self.screen_corners) < 4:
                self.screen_corners.append((orig_x, orig_y))
                print(f"✓ 标注角点 {len(self.screen_corners)}: ({orig_x}, {orig_y})")
                
                if len(self.screen_corners) == 4:
                    print("✓ 屏幕四个角点标注完成，正在生成网格和边界框...")
                    self._generate_grid()
                    bbox_info = self._calculate_screen_bbox()
                    if bbox_info:
                        print(f"  边界框: {bbox_info['xyxy_bbox']}")
                        print(f"  YOLO格式: {[f'{x:.4f}' for x in bbox_info['yolo_bbox']]}")
    
    def _generate_grid(self):
        """基于四个角点生成 6x6 网格"""
        if len(self.screen_corners) != 4:
            return
        
        corners = self._sort_corners(self.screen_corners)
        
        self.grid_points = []
        for i in range(self.grid_rows + 1):
            for j in range(self.grid_cols + 1):
                t_vertical = i / self.grid_rows
                t_horizontal = j / self.grid_cols
                
                top_point = self._interpolate_point(corners[0], corners[1], t_horizontal)
                bottom_point = self._interpolate_point(corners[3], corners[2], t_horizontal)
                
                grid_point = self._interpolate_point(top_point, bottom_point, t_vertical)
                self.grid_points.append(grid_point)
        
        print(f"  生成 {len(self.grid_points)} 个网格点")
    
    def _sort_corners(self, corners: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """按左上、右上、右下、左下的顺序排列角点"""
        cx = sum(p[0] for p in corners) / 4
        cy = sum(p[1] for p in corners) / 4
        
        top_left = min(corners, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2 
                      if p[0] < cx and p[1] < cy else float('inf'))
        top_right = min(corners, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2 
                       if p[0] > cx and p[1] < cy else float('inf'))
        bottom_right = min(corners, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2 
                          if p[0] > cx and p[1] > cy else float('inf'))
        bottom_left = min(corners, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2 
                         if p[0] < cx and p[1] > cy else float('inf'))
        
        return [top_left, top_right, bottom_right, bottom_left]
    
    def _interpolate_point(self, p1: Tuple[int, int], p2: Tuple[int, int], t: float) -> Tuple[int, int]:
        """在两点间进行线性插值"""
        x = int(p1[0] + t * (p2[0] - p1[0]))
        y = int(p1[1] + t * (p2[1] - p1[1]))
        return (x, y)
    
    def _draw_annotations(self, image: np.ndarray, for_display: bool = True) -> np.ndarray:
        """在图像上绘制标注信息"""
        display_img = image.copy()
        
        if for_display:
            height, width = display_img.shape[:2]
            display_height = int(height * self.display_scale)
            display_width = int(width * self.display_scale)
            display_img = cv2.resize(display_img, (display_width, display_height))
            scale_factor = self.display_scale
        else:
            scale_factor = 1.0
        
        # 绘制屏幕边界框
        if len(self.screen_corners) == 4:
            bbox_info = self._calculate_screen_bbox()
            if bbox_info:
                x_min, y_min, x_max, y_max = bbox_info['xyxy_bbox']
                pt1 = (int(x_min * scale_factor), int(y_min * scale_factor))
                pt2 = (int(x_max * scale_factor), int(y_max * scale_factor))
                thickness = max(2, int(3 * scale_factor))
                cv2.rectangle(display_img, pt1, pt2, (0, 0, 255), thickness)
                
                # 添加标签
                label = "Screen"
                font_scale = 0.8 * scale_factor if scale_factor < 1.0 else 0.8
                label_thickness = max(1, int(2 * scale_factor))
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, label_thickness)
                
                cv2.rectangle(display_img, pt1, (pt1[0] + label_w + 10, pt1[1] - label_h - 10), (0, 0, 255), -1)
                cv2.putText(display_img, label, (pt1[0] + 5, pt1[1] - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), label_thickness)
        
        # 绘制角点
        for i, corner in enumerate(self.screen_corners):
            display_corner = (int(corner[0] * scale_factor), int(corner[1] * scale_factor))
            radius = int(self.point_radius * scale_factor) if scale_factor < 1.0 else self.point_radius
            cv2.circle(display_img, display_corner, radius, (0, 255, 0), -1)
            
            font_scale = 0.6 * scale_factor if scale_factor < 1.0 else 0.6
            thickness = max(1, int(2 * scale_factor))
            cv2.putText(display_img, str(i+1), 
                       (display_corner[0] + 10, display_corner[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
        
        # 绘制边框连线
        if len(self.screen_corners) >= 2:
            for i in range(len(self.screen_corners)):
                pt1 = (int(self.screen_corners[i][0] * scale_factor),
                      int(self.screen_corners[i][1] * scale_factor))
                pt2 = (int(self.screen_corners[(i+1) % len(self.screen_corners)][0] * scale_factor),
                      int(self.screen_corners[(i+1) % len(self.screen_corners)][1] * scale_factor))
                thickness = max(1, int(self.line_thickness * scale_factor))
                cv2.line(display_img, pt1, pt2, (255, 0, 0), thickness)
        
        # 绘制网格
        if len(self.grid_points) > 0:
            line_thickness = max(1, int(1 * scale_factor))
            # 水平线
            for i in range(self.grid_rows + 1):
                for j in range(self.grid_cols):
                    idx1 = i * (self.grid_cols + 1) + j
                    idx2 = i * (self.grid_cols + 1) + j + 1
                    if idx1 < len(self.grid_points) and idx2 < len(self.grid_points):
                        pt1 = (int(self.grid_points[idx1][0] * scale_factor),
                              int(self.grid_points[idx1][1] * scale_factor))
                        pt2 = (int(self.grid_points[idx2][0] * scale_factor),
                              int(self.grid_points[idx2][1] * scale_factor))
                        cv2.line(display_img, pt1, pt2, (0, 255, 255), line_thickness)
            
            # 垂直线
            for i in range(self.grid_rows):
                for j in range(self.grid_cols + 1):
                    idx1 = i * (self.grid_cols + 1) + j
                    idx2 = (i + 1) * (self.grid_cols + 1) + j
                    if idx1 < len(self.grid_points) and idx2 < len(self.grid_points):
                        pt1 = (int(self.grid_points[idx1][0] * scale_factor),
                              int(self.grid_points[idx1][1] * scale_factor))
                        pt2 = (int(self.grid_points[idx2][0] * scale_factor),
                              int(self.grid_points[idx2][1] * scale_factor))
                        cv2.line(display_img, pt1, pt2, (0, 255, 255), line_thickness)
            
            # 绘制网格编号
            for i in range(self.grid_rows):
                for j in range(self.grid_cols):
                    tl_idx = i * (self.grid_cols + 1) + j
                    tr_idx = i * (self.grid_cols + 1) + j + 1
                    bl_idx = (i + 1) * (self.grid_cols + 1) + j
                    br_idx = (i + 1) * (self.grid_cols + 1) + j + 1
                    
                    if all(idx < len(self.grid_points) for idx in [tl_idx, tr_idx, bl_idx, br_idx]):
                        center_x = (self.grid_points[tl_idx][0] + self.grid_points[tr_idx][0] + 
                                   self.grid_points[bl_idx][0] + self.grid_points[br_idx][0]) // 4
                        center_y = (self.grid_points[tl_idx][1] + self.grid_points[tr_idx][1] + 
                                   self.grid_points[bl_idx][1] + self.grid_points[br_idx][1]) // 4
                        
                        display_center = (int(center_x * scale_factor), int(center_y * scale_factor))
                        grid_id = i * self.grid_cols + j
                        
                        if for_display:
                            font_scale = 0.4 * scale_factor if scale_factor < 1.0 else 0.4
                        else:
                            font_scale = 0.8 * min(self.width/1920, self.height/1080)
                        
                        thickness = max(1, int(1 * scale_factor))
                        cv2.putText(display_img, str(grid_id), display_center,
                                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), thickness)
        
        return display_img
    
    def _save_yolo_annotation(self, sample_id: str, bbox_info: Dict[str, Any]):
        """保存YOLO格式的标注文件"""
        yolo_path = self.output_dir / "yolo_labels" / f"{sample_id}.txt"
        yolo_bbox = bbox_info['yolo_bbox']
        class_id = 0
        
        with open(yolo_path, 'w', encoding='utf-8') as f:
            f.write(f"{class_id} {yolo_bbox[0]:.6f} {yolo_bbox[1]:.6f} {yolo_bbox[2]:.6f} {yolo_bbox[3]:.6f}\n")
    
    def _save_coco_annotation(self, sample_id: str, bbox_info: Dict[str, Any]):
        """保存COCO格式的标注文件"""
        coco_path = self.output_dir / "coco_labels" / f"{sample_id}.json"
        
        coco_annotation = {
            "image": {
                "id": self.current_sample_id,
                "file_name": f"{sample_id}_color.png",
                "width": self.width,
                "height": self.height
            },
            "annotations": [
                {
                    "id": 1,
                    "image_id": self.current_sample_id,
                    "category_id": 1,
                    "bbox": bbox_info['coco_bbox'],
                    "area": bbox_info['area'],
                    "iscrowd": 0,
                    "segmentation": [
                        [coord for point in bbox_info['polygon_corners'] for coord in point]
                    ]
                }
            ],
            "categories": [
                {
                    "id": 1,
                    "name": "screen",
                    "supercategory": "object"
                }
            ]
        }
        
        with open(coco_path, 'w', encoding='utf-8') as f:
            json.dump(coco_annotation, f, indent=2, ensure_ascii=False)
    
    def save_annotation(self, color_image: np.ndarray, depth_raw: np.ndarray, depth_colored: np.ndarray):
        """保存当前标注数据"""
        if len(self.screen_corners) != 4:
            print("❌ 错误: 需要标注4个角点才能保存")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        sample_id = f"{self.current_sample_id:04d}_{timestamp}"
        
        bbox_info = self._calculate_screen_bbox()
        if not bbox_info:
            print("❌ 错误: 无法计算边界框")
            return
        
        # 保存图像
        color_path = self.output_dir / "images" / f"{sample_id}_color.png"
        depth_path = self.output_dir / "depth" / f"{sample_id}_depth.png"
        depth_colored_path = self.output_dir / "depth_colored" / f"{sample_id}_depth_colored.png"
        
        cv2.imwrite(str(color_path), color_image)
        cv2.imwrite(str(depth_path), depth_raw)
        cv2.imwrite(str(depth_colored_path), depth_colored)
        
        # 保存标注
        self._save_yolo_annotation(sample_id, bbox_info)
        self._save_coco_annotation(sample_id, bbox_info)
        
        # 生成网格区域信息
        grid_regions = []
        for i in range(self.grid_rows):
            for j in range(self.grid_cols):
                tl_idx = i * (self.grid_cols + 1) + j
                tr_idx = i * (self.grid_cols + 1) + j + 1
                bl_idx = (i + 1) * (self.grid_cols + 1) + j
                br_idx = (i + 1) * (self.grid_cols + 1) + j + 1
                
                if all(idx < len(self.grid_points) for idx in [tl_idx, tr_idx, bl_idx, br_idx]):
                    region = {
                        "grid_id": i * self.grid_cols + j,
                        "row": i,
                        "col": j,
                        "corners": [
                            self.grid_points[tl_idx],
                            self.grid_points[tr_idx],
                            self.grid_points[br_idx],
                            self.grid_points[bl_idx]
                        ]
                    }
                    grid_regions.append(region)
        
        # 保存完整标注数据
        annotation_data = {
            "sample_id": sample_id,
            "timestamp": timestamp,
            "resolution": self.resolution_name,
            "color_image_path": str(color_path.relative_to(self.output_dir)),
            "depth_image_path": str(depth_path.relative_to(self.output_dir)),
            "depth_colored_path": str(depth_colored_path.relative_to(self.output_dir)),
            "screen_detection": {
                "class_name": "screen",
                "class_id": 0,
                "bbox_formats": bbox_info,
                "confidence": 1.0
            },
            "screen_corners": self.screen_corners,
            "grid_size": [self.grid_rows, self.grid_cols],
            "grid_regions": grid_regions,
            "color_image_size": [color_image.shape[1], color_image.shape[0]],
            "depth_image_size": [depth_raw.shape[1], depth_raw.shape[0]],
            "depth_scale": float(self.camera.depth_scale),
            "depth_unit": "depth_sensor_units",
            "depth_conversion": f"depth_value * {self.camera.depth_scale} = meters",
        }
        
        annotation_path = self.output_dir / "annotations" / f"{sample_id}.json"
        with open(annotation_path, 'w', encoding='utf-8') as f:
            json.dump(annotation_data, f, indent=2, ensure_ascii=False)
        
        # 保存网格可视化
        annotated_image = self._draw_annotations(color_image, for_display=False)
        grid_vis_path = self.output_dir / "grids" / f"{sample_id}_grid.png"
        cv2.imwrite(str(grid_vis_path), annotated_image)
        
        print(f"\n✅ 数据已保存: {sample_id}")
        print(f"   - 彩色图像: {color_path.name}")
        print(f"   - 深度图像: {depth_path.name}")
        print(f"   - 深度可视化: {depth_colored_path.name}")
        print(f"   - 标注数据: {annotation_path.name}")
        print(f"   - 网格可视化: {grid_vis_path.name}")
        print(f"   - 边界框: {bbox_info['xyxy_bbox']}")
        print(f"   - 深度尺度: {self.camera.depth_scale}\n")
        
        self.current_sample_id += 1
        self.screen_corners = []
        self.grid_points = []
    
    def run_data_collection(self):
        """运行数据采集主循环"""
        print(f"\n🚀 启动屏幕检测数据采集器 ({self.resolution_name})")
        print("\n操作说明:")
        print("  A - 开始标注模式")
        print("  S - 保存数据 (标注完成后)")
        print("  R - 重置当前标注")
        print("  D - 切换显示深度图像")
        print("  Q/ESC - 退出")
        print("=" * 60)
        
        # 启动摄像头
        if not self.camera.start():
            print("❌ 摄像头启动失败")
            return
        
        window_name = f"Screen Detection Collector - {self.resolution_name}"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        show_depth = False
        
        try:
            while True:
                rgb, depth_raw, depth_colored = self.camera.get_frames()
                
                if rgb is not None and depth_raw is not None:
                    self.current_color_image = rgb
                    self.current_depth_raw = depth_raw
                    self.current_depth_colored = depth_colored
                    
                    # 选择显示图像
                    if show_depth and depth_colored is not None:
                        display_source = depth_colored
                        image_type = "深度"
                    else:
                        display_source = rgb
                        image_type = "彩色"
                    
                    # 绘制标注
                    display_image = self._draw_annotations(display_source, for_display=True)
                    
                    # 添加状态信息
                    font_scale = 0.5 * min(self.width/1920, self.height/1080) * self.display_scale
                    font_scale = max(0.4, min(0.8, font_scale))
                    
                    status_y = 30
                    cv2.putText(display_image, f"分辨率: {self.resolution_name} | 显示: {image_type}", 
                               (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
                    
                    status_y += 25
                    if self.is_annotating:
                        status = f"标注模式 - 已标注 {len(self.screen_corners)}/4 个角点"
                        if len(self.screen_corners) == 4:
                            status += " - 按 S 保存"
                    else:
                        status = "按 A 开始标注"
                    cv2.putText(display_image, status, (10, status_y),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
                    
                    status_y += 25
                    cv2.putText(display_image, f"已采集样本: {self.current_sample_id}", 
                               (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
                    
                    cv2.imshow(window_name, display_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
                elif key == ord('a'):
                    if not self.is_annotating:
                        self.is_annotating = True
                        self.screen_corners = []
                        self.grid_points = []
                        print("📝 开始标注模式")
                elif key == ord('s'):
                    if (self.is_annotating and len(self.screen_corners) == 4 and 
                        self.current_color_image is not None):
                        self.save_annotation(self.current_color_image, 
                                           self.current_depth_raw, 
                                           self.current_depth_colored)
                        self.is_annotating = False
                elif key == ord('r'):
                    self.screen_corners = []
                    self.grid_points = []
                    print("🔄 重置标注")
                elif key == ord('d'):
                    show_depth = not show_depth
                    print(f"🔄 切换显示: {'深度' if show_depth else '彩色'}")
                
        except KeyboardInterrupt:
            print("\n🛑 接收到中断信号")
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
            print("👋 数据采集结束")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="屏幕检测数据采集器")
    parser.add_argument("--output-dir", type=str, default="data/screen_detection",
                       help="数据保存根目录")
    parser.add_argument("--resolution", type=str, default="1920x1080",
                       choices=list(SUPPORTED_RESOLUTIONS.keys()),
                       help="采集分辨率")
    parser.add_argument("--width", type=int, help="自定义宽度")
    parser.add_argument("--height", type=int, help="自定义高度")
    
    args = parser.parse_args()
    
    if args.width and args.height:
        width, height = args.width, args.height
    else:
        width, height = SUPPORTED_RESOLUTIONS[args.resolution]
    
    print(f"\n📊 数据采集配置:")
    print(f"   分辨率: {width}x{height}")
    print(f"   输出目录: {args.output_dir}/{width}x{height}/")
    
    collector = ScreenDetectionDataCollector(args.output_dir, width, height)
    collector.run_data_collection()


if __name__ == "__main__":
    main()