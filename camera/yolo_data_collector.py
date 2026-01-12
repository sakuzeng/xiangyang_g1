#!/usr/bin/env python3
"""
screen_detection_data_collector.py
================================

屏幕分割数据采集器 - YOLO Segmentation 格式

功能：
1. 使用 RealSense 摄像头采集屏幕图像
2. 交互式标注屏幕四角点
3. 保存 YOLO 分割格式标注 (归一化多边形坐标)
4. 所有数据统一保存,后续通过专门脚本划分训练集/验证集

标注格式:
0 x1 y1 x2 y2 x3 y3 x4 y4
(class_id + 4个角点的归一化坐标)

使用方法:
python screen_detection_data_collector.py --resolution 960x540 --output-dir my_dataset/seg_960x540
"""

import os
import sys
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

# 导入封装好的 RealSense 摄像头类
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'loco', 'unitree_sdk_python'))
from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera


# 支持的分辨率配置
SUPPORTED_RESOLUTIONS = {
    "1920x1080": (1920, 1080),
    "1280x720": (1280, 720),
    "960x540": (960, 540),
    "848x480": (848, 480),
    "640x480": (640, 480),
}


class ScreenSegmentationCollector:
    """屏幕分割数据采集器 - YOLO Segmentation 格式"""
    
    def __init__(self, output_dir: str, width: int = 960, height: int = 540):
        """
        初始化数据采集器
        
        Args:
            output_dir: 数据集根目录
            width: 图像宽度
            height: 图像高度
        """
        self.width = width
        self.height = height
        self.output_dir = Path(output_dir)
        
        # 创建简单的二级目录结构
        self.images_dir = self.output_dir / "images"
        self.labels_dir = self.output_dir / "labels"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        
        # 创建 dataset.yaml (如果不存在)
        self._create_dataset_yaml()
        
        # 标注状态
        self.current_color_image = None
        self.screen_corners = []  # 屏幕四角点 (像素坐标)
        self.is_annotating = False
        self.sample_count = 0
        
        # 显示参数
        self.display_scale = self._get_display_scale()
        
        # 初始化摄像头
        self.camera = RealSenseCamera(width=width, height=height, fps=30)
        
        print(f"✅ 数据采集器初始化完成")
        print(f"   分辨率: {width}x{height}")
        print(f"   数据集目录: {self.output_dir}")
    
    def _create_dataset_yaml(self):
        """创建 YOLO 数据集配置文件"""
        yaml_path = self.output_dir / "dataset.yaml"
        if not yaml_path.exists():
            yaml_content = f"""# YOLO11 Segmentation Dataset Configuration
# 使用前请先运行数据集划分脚本将 images/ 和 labels/ 分为 train/val/test

path: {self.output_dir.absolute()}
train: images/train
val: images/val
test: images/test

nc: 1  # number of classes
names: ['screen']  # class names
"""
            with open(yaml_path, 'w', encoding='utf-8') as f:
                f.write(yaml_content)
            print(f"✅ 已创建 dataset.yaml")
    
    def _get_display_scale(self) -> float:
        """根据分辨率自动计算显示缩放比例"""
        max_display_width = 1200
        max_display_height = 900
        scale_x = max_display_width / self.width
        scale_y = max_display_height / self.height
        return min(scale_x, scale_y, 1.0)
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数 - 标注屏幕四角点"""
        if not self.is_annotating or self.current_color_image is None:
            return
        
        if event == cv2.EVENT_LBUTTONDOWN and len(self.screen_corners) < 4:
            # 转换回原始图像坐标
            orig_x = int(x / self.display_scale)
            orig_y = int(y / self.display_scale)
            
            self.screen_corners.append((orig_x, orig_y))
            print(f"✓ 标注角点 {len(self.screen_corners)}/4: ({orig_x}, {orig_y})")
            
            if len(self.screen_corners) == 4:
                # 计算归一化坐标
                sorted_corners = self._sort_corners(self.screen_corners)
                norm_coords = self._normalize_corners(sorted_corners)
                print(f"✓ 归一化坐标: {' '.join([f'{c:.6f}' for c in norm_coords])}")
                print("📝 按 S 保存 | 按 R 重新标注")
    
    def _normalize_corners(self, corners: List[Tuple[int, int]]) -> List[float]:
        """将像素坐标归一化到 [0, 1]"""
        normalized = []
        for x, y in corners:
            normalized.extend([x / self.width, y / self.height])
        return normalized
    
    def _sort_corners(self, corners: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """按左上、右上、右下、左下顺序排列角点"""
        cx = sum(p[0] for p in corners) / 4
        cy = sum(p[1] for p in corners) / 4
        
        # 根据象限分类
        sorted_corners = []
        # 左上
        sorted_corners.append(min([p for p in corners if p[0] < cx and p[1] < cy], 
                                 key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2, 
                                 default=min(corners, key=lambda p: p[0] + p[1])))
        # 右上
        sorted_corners.append(min([p for p in corners if p[0] > cx and p[1] < cy], 
                                 key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2, 
                                 default=max(corners, key=lambda p: p[0] - p[1])))
        # 右下
        sorted_corners.append(min([p for p in corners if p[0] > cx and p[1] > cy], 
                                 key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2, 
                                 default=max(corners, key=lambda p: p[0] + p[1])))
        # 左下
        sorted_corners.append(min([p for p in corners if p[0] < cx and p[1] > cy], 
                                 key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2, 
                                 default=min(corners, key=lambda p: p[1] - p[0])))
        
        return sorted_corners
    
    def _draw_annotations(self, image: np.ndarray) -> np.ndarray:
        """在图像上绘制标注信息"""
        display_img = image.copy()
        
        # 缩放显示
        h, w = display_img.shape[:2]
        display_h = int(h * self.display_scale)
        display_w = int(w * self.display_scale)
        display_img = cv2.resize(display_img, (display_w, display_h))
        
        # 绘制已标注的角点
        for i, corner in enumerate(self.screen_corners):
            pt = (int(corner[0] * self.display_scale), int(corner[1] * self.display_scale))
            cv2.circle(display_img, pt, 5, (0, 255, 0), -1)
            cv2.putText(display_img, str(i+1), (pt[0]+10, pt[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 绘制多边形连线
        if len(self.screen_corners) >= 2:
            scaled_corners = [(int(x * self.display_scale), int(y * self.display_scale)) 
                            for x, y in self.screen_corners]
            for i in range(len(scaled_corners)):
                pt1 = scaled_corners[i]
                pt2 = scaled_corners[(i+1) % len(scaled_corners)]
                cv2.line(display_img, pt1, pt2, (255, 0, 0), 2)
        
        # 填充多边形 (半透明)
        if len(self.screen_corners) == 4:
            overlay = display_img.copy()
            scaled_corners = np.array([(int(x * self.display_scale), int(y * self.display_scale)) 
                                     for x, y in self.screen_corners], dtype=np.int32)
            cv2.fillPoly(overlay, [scaled_corners], (0, 255, 255))
            cv2.addWeighted(overlay, 0.2, display_img, 0.8, 0, display_img)
        
        # 显示状态信息
        font_scale = 0.6
        status_y = 30
        cv2.putText(display_img, f"分辨率: {self.width}x{self.height}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
        
        status_y += 30
        if self.is_annotating:
            status = f"标注模式 - 已标注 {len(self.screen_corners)}/4 个角点"
            if len(self.screen_corners) == 4:
                status += " [按 S 保存]"
        else:
            status = "按 A 开始标注"
        cv2.putText(display_img, status, (10, status_y),
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
        
        status_y += 30
        cv2.putText(display_img, f"已采集: {self.sample_count} 个样本", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
        
        return display_img
    
    def save_sample(self):
        """保存当前标注样本"""
        if len(self.screen_corners) != 4:
            print("❌ 错误: 需要标注4个角点才能保存")
            return
        
        if self.current_color_image is None:
            print("❌ 错误: 无图像数据")
            return
        
        # 排序角点 (左上、右上、右下、左下)
        sorted_corners = self._sort_corners(self.screen_corners)
        
        # 生成文件名 (4位编号)
        sample_id = f"{self.sample_count:04d}"
        
        # 保存图像
        image_path = self.images_dir / f"{sample_id}.png"
        cv2.imwrite(str(image_path), self.current_color_image)
        
        # 保存 YOLO 分割标注
        label_path = self.labels_dir / f"{sample_id}.txt"
        norm_coords = self._normalize_corners(sorted_corners)
        
        with open(label_path, 'w', encoding='utf-8') as f:
            # 格式: class_id x1 y1 x2 y2 x3 y3 x4 y4
            line = "0 " + " ".join([f"{c:.6f}" for c in norm_coords])
            f.write(line + "\n")
        
        print(f"\n✅ 样本已保存: {sample_id}")
        print(f"   图像: {image_path.name}")
        print(f"   标注: {label_path.name}")
        print(f"   内容: {line}\n")
        
        # 更新计数并重置
        self.sample_count += 1
        self.screen_corners = []
        self.is_annotating = False
    
    def run(self):
        """运行数据采集主循环"""
        print(f"\n🚀 启动屏幕分割数据采集器")
        print("\n操作说明:")
        print("  A - 开始标注模式 (按顺序点击屏幕四个角点)")
        print("  S - 保存当前标注")
        print("  R - 重置当前标注")
        print("  Q/ESC - 退出")
        print("=" * 60)
        
        # 启动摄像头
        if not self.camera.start():
            print("❌ 摄像头启动失败")
            return
        
        window_name = f"Screen Segmentation Collector - {self.width}x{self.height}"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        try:
            while True:
                # 获取图像 (只需要彩色图)
                rgb, _, _ = self.camera.get_frames()
                
                if rgb is not None:
                    self.current_color_image = rgb
                    display_image = self._draw_annotations(rgb)
                    cv2.imshow(window_name, display_image)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):  # ESC or Q
                    break
                elif key == ord('a'):  # 开始标注
                    if not self.is_annotating:
                        self.is_annotating = True
                        self.screen_corners = []
                        print("📝 开始标注模式 - 按顺序点击屏幕四个角点")
                elif key == ord('s'):  # 保存
                    if self.is_annotating and len(self.screen_corners) == 4:
                        self.save_sample()
                elif key == ord('r'):  # 重置
                    self.screen_corners = []
                    self.is_annotating = False
                    print("🔄 重置标注")
                
        except KeyboardInterrupt:
            print("\n🛑 接收到中断信号")
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
            print(f"\n📊 采集统计: 共 {self.sample_count} 个样本")
            print("👋 数据采集结束")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="屏幕分割数据采集器 (YOLO格式)")
    parser.add_argument("--output-dir", type=str, default="my_dataset/seg_960x540",
                       help="数据集根目录")
    parser.add_argument("--resolution", type=str, default="960x540",
                       choices=list(SUPPORTED_RESOLUTIONS.keys()),
                       help="采集分辨率")
    parser.add_argument("--width", type=int, help="自定义宽度")
    parser.add_argument("--height", type=int, help="自定义高度")
    
    args = parser.parse_args()
    
    # 解析分辨率
    if args.width and args.height:
        width, height = args.width, args.height
    else:
        width, height = SUPPORTED_RESOLUTIONS[args.resolution]
    
    print(f"\n📊 数据采集配置:")
    print(f"   分辨率: {width}x{height}")
    print(f"   输出目录: {args.output_dir}")
    
    collector = ScreenSegmentationCollector(args.output_dir, width, height)
    collector.run()


if __name__ == "__main__":
    main()