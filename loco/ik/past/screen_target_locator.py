#!/usr/bin/env python3
"""
screen_target_locator.py
========================

基于YOLO检测的屏幕目标定位系统

功能:
1. 调用外部YOLO服务识别屏幕区域
2. 根据指定编号获取目标区域中心点
3. 将像素坐标转换为机器人Torso坐标系
4. 支持实时预览和交互式操作

依赖:
- RealSenseCamera (摄像头接口)
- requests (HTTP请求)
- scipy (坐标转换)
"""

import sys
import os
import cv2
import numpy as np
import requests
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from typing import Optional, Tuple, Dict, Any
from pathlib import Path

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'loco', 'unitree_sdk_python'))

from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera


# ==========================================
# 坐标转换器 (从 camera_to_torso.py 移植)
# ==========================================
class CoordTransformer:
    """相机坐标系 -> Torso坐标系转换器"""
    
    def __init__(self):
        # URDF 原始平移参数
        self.urdf_trans = np.array([0.0576235, 0.01753, 0.42987])
        
        # 校准好的 Pitch
        self.pitch_offset = 0.23
        self.base_pitch = 0.8307767239493009
        
        self._recalc_matrices()
    
    def _recalc_matrices(self):
        """重新计算旋转矩阵"""
        final_pitch = self.base_pitch + self.pitch_offset
        
        # Optical -> Link
        self.mat_opt_to_link = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])
        
        # Link -> Torso
        self.urdf_rpy = [0, final_pitch, 0]
        r_obj = R.from_euler('xyz', self.urdf_rpy, degrees=False)
        try:
            self.mat_link_to_torso = r_obj.as_matrix()
        except:
            self.mat_link_to_torso = r_obj.as_dcm()
    
    def process(self, point_cam_optical: np.ndarray) -> np.ndarray:
        """
        将相机光学坐标系的点转换到Torso坐标系
        
        Args:
            point_cam_optical: 相机坐标系下的3D点 [x, y, z]
        
        Returns:
            np.ndarray: Torso坐标系下的3D点 [x, y, z]
        """
        P_opt = np.array(point_cam_optical)
        P_link = self.mat_opt_to_link @ P_opt
        P_torso = self.mat_link_to_torso @ P_link + self.urdf_trans
        return P_torso


# ==========================================
# YOLO 服务客户端
# ==========================================
class YOLOClient:
    """YOLO屏幕检测服务客户端"""
    
    def __init__(self, server_url: str = "http://192.168.77.103:28000"):
        self.server_url = server_url.rstrip('/')
        self.endpoint = f"{self.server_url}/yolo"
    
    def detect_screen_target(self, image: np.ndarray, target_index: int) -> Optional[Dict[str, Any]]:
        """
        调用YOLO服务检测屏幕并返回目标区域信息
        
        Args:
            image: BGR图像
            target_index: 目标区域编号 (0-35)
        
        Returns:
            Dict包含:
                - found: bool
                - screen_corners: [[x,y], ...]  # 屏幕四角点
                - target_region: {
                    "center": (x, y),
                    "corners": [[x,y], ...]
                  }
        """
        try:
            # 编码图像
            _, img_encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            # 发送请求
            files = {'file': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')}
            data = {'target_index': target_index}
            
            response = requests.post(
                self.endpoint,
                files=files,
                data=data,
                timeout=5.0
            )
            
            if response.status_code == 200:
                result = response.json()
                return result
            else:
                print(f"❌ YOLO服务错误: {response.status_code}")
                return None
                
        except requests.exceptions.Timeout:
            print("❌ YOLO服务超时")
            return None
        except Exception as e:
            print(f"❌ YOLO服务调用失败: {e}")
            return None


# ==========================================
# 深度辅助工具 (从 camera_to_torso.py 移植)
# ==========================================
class DepthHelper:
    """深度图辅助工具"""
    
    @staticmethod
    def get_precise_depth(depth_image: np.ndarray, x: int, y: int, 
                         max_search_radius: int = 20) -> Tuple[float, Tuple[int, int]]:
        """
        智能深度搜索
        
        Returns:
            (depth_value, (offset_x, offset_y))
        """
        height, width = depth_image.shape
        
        # 边界检查
        if not (0 <= x < width and 0 <= y < height):
            return 0, (0, 0)
        
        # 策略1: 直接读取中心像素
        center_depth = depth_image[y, x]
        if center_depth > 0:
            return center_depth, (0, 0)
        
        # 策略2: 同心圆扩散搜索
        for radius in range(1, max_search_radius + 1):
            candidates = []
            num_samples = max(8, radius * 2)
            
            for i in range(num_samples):
                angle = 2 * np.pi * i / num_samples
                dx = int(radius * np.cos(angle))
                dy = int(radius * np.sin(angle))
                
                nx, ny = x + dx, y + dy
                
                if 0 <= nx < width and 0 <= ny < height:
                    depth_val = depth_image[ny, nx]
                    if depth_val > 0:
                        candidates.append((depth_val, dx, dy, radius))
            
            if candidates:
                depths = [c[0] for c in candidates]
                median_depth = np.median(depths)
                best_candidate = min(candidates, key=lambda c: abs(c[0] - median_depth))
                depth_val, dx, dy, r = best_candidate
                return depth_val, (dx, dy)
        
        # 策略3: 区域中值填补
        x_min = max(0, x - max_search_radius)
        x_max = min(width, x + max_search_radius + 1)
        y_min = max(0, y - max_search_radius)
        y_max = min(height, y + max_search_radius + 1)
        
        roi = depth_image[y_min:y_max, x_min:x_max]
        valid_pixels = roi[roi > 0]
        
        if len(valid_pixels) > 10:
            median_val = np.median(valid_pixels)
            return median_val, (0, 0)
        
        return 0, (0, 0)


# ==========================================
# 主应用类
# ==========================================
class ScreenTargetLocator:
    """屏幕目标定位器"""
    
    def __init__(self, yolo_server_url: str = "http://192.168.77.103:28000"):
        # 组件初始化
        self.camera = RealSenseCamera(width=848, height=480, fps=30)
        self.yolo_client = YOLOClient(yolo_server_url)
        self.coord_transformer = CoordTransformer()
        self.depth_helper = DepthHelper()
        
        # 状态变量
        self.current_target_index = 0  # 默认目标编号
        self.last_detection_result = None
        self.last_torso_coords = None
        
        print(f"✅ 屏幕目标定位器初始化完成")
        print(f"   YOLO服务: {yolo_server_url}")
        print(f"   摄像头分辨率: 848x480")
    
    def detect_and_locate(self, color_image: np.ndarray, depth_raw: np.ndarray, 
                         target_index: int) -> Optional[Dict[str, Any]]:
        """
        检测屏幕并定位目标区域的Torso坐标
        
        Args:
            color_image: 彩色图像
            depth_raw: 原始深度图
            target_index: 目标区域编号 (0-35)
        
        Returns:
            Dict包含:
                - target_index: int
                - pixel_coord: (x, y)  # 像素坐标
                - depth_meters: float
                - camera_coord: [x, y, z]  # 相机坐标系
                - torso_coord: [x, y, z]   # Torso坐标系
                - screen_corners: [[x,y], ...]
                - target_region: {...}
        """
        # 1. 调用YOLO服务
        yolo_result = self.yolo_client.detect_screen_target(color_image, target_index)
        
        if not yolo_result or not yolo_result.get('found'):
            print(f"❌ 未检测到屏幕或目标区域")
            return None
        
        # 2. 提取目标中心点像素坐标
        target_center = yolo_result['target_region']['center']
        pixel_x, pixel_y = target_center
        
        print(f"\n📍 目标区域 {target_index} 中心: ({pixel_x}, {pixel_y})")
        
        # 3. 获取深度值
        depth_value, search_offset = self.depth_helper.get_precise_depth(
            depth_raw, pixel_x, pixel_y, max_search_radius=20
        )
        
        if depth_value == 0:
            print(f"❌ 无法获取有效深度值")
            return None
        
        # 4. 转换为米制
        depth_meters = depth_value * self.camera.depth_scale
        
        if search_offset != (0, 0):
            print(f"🔍 深度搜索偏移: {search_offset}, 深度: {depth_meters:.3f}m")
        else:
            print(f"📏 深度: {depth_meters:.3f}m")
        
        # 5. 反投影到相机坐标系
        actual_pixel_x = pixel_x + search_offset[0]
        actual_pixel_y = pixel_y + search_offset[1]
        
        camera_point = rs.rs2_deproject_pixel_to_point(
            self.camera.depth_intrinsics,
            [actual_pixel_x, actual_pixel_y],
            depth_meters
        )
        
        # 6. 转换到Torso坐标系
        torso_point = self.coord_transformer.process(np.array(camera_point))
        
        print(f"📷 相机坐标: X={camera_point[0]:.3f}, Y={camera_point[1]:.3f}, Z={camera_point[2]:.3f}")
        print(f"🤖 Torso坐标: X={torso_point[0]:.3f}, Y={torso_point[1]:.3f}, Z={torso_point[2]:.3f}")
        
        return {
            'target_index': target_index,
            'pixel_coord': (pixel_x, pixel_y),
            'actual_pixel_coord': (actual_pixel_x, actual_pixel_y),
            'depth_meters': float(depth_meters),
            'camera_coord': list(camera_point),  # ✅ 修复: camera_point 已经是 list
            'torso_coord': torso_point.tolist(),  # ✅ torso_point 是 ndarray,需要转换
            'screen_corners': yolo_result['screen_corners'],
            'target_region': yolo_result['target_region'],
            'search_offset': search_offset
        }
    
    def visualize_result(self, color_image: np.ndarray, result: Dict[str, Any]) -> np.ndarray:
        """在图像上可视化检测结果"""
        vis_image = color_image.copy()
        
        # 绘制屏幕四角点
        screen_corners = result['screen_corners']
        for i, corner in enumerate(screen_corners):
            cv2.circle(vis_image, tuple(corner), 5, (0, 255, 0), -1)
            cv2.putText(vis_image, str(i+1), (corner[0]+10, corner[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 绘制屏幕边框
        for i in range(4):
            pt1 = tuple(screen_corners[i])
            pt2 = tuple(screen_corners[(i+1) % 4])
            cv2.line(vis_image, pt1, pt2, (255, 0, 0), 2)
        
        # 绘制目标区域
        target_corners = result['target_region']['corners']
        for i in range(4):
            pt1 = tuple(target_corners[i])
            pt2 = tuple(target_corners[(i+1) % 4])
            cv2.line(vis_image, pt1, pt2, (0, 255, 255), 2)
        
        # 绘制目标中心点
        center = result['pixel_coord']
        actual_center = result['actual_pixel_coord']
        
        cv2.drawMarker(vis_image, center, (0, 255, 255), 
                      cv2.MARKER_TILTED_CROSS, 15, 2)
        
        if result['search_offset'] != (0, 0):
            cv2.circle(vis_image, actual_center, 5, (0, 0, 255), -1)
            cv2.line(vis_image, center, actual_center, (255, 255, 0), 1)
        else:
            cv2.circle(vis_image, center, 5, (0, 255, 0), -1)
        
        # 显示Torso坐标
        torso = result['torso_coord']
        label = f"Grid{result['target_index']}: X={torso[0]:.2f} Y={torso[1]:.2f} Z={torso[2]:.2f}"
        cv2.putText(vis_image, label, (actual_center[0]+12, actual_center[1]-5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return vis_image
    
    def run_interactive(self):
        """运行交互式定位模式"""
        print("\n🚀 启动屏幕目标定位器")
        print("\n操作说明:")
        print("  0-9 - 快速选择目标编号 (0-9)")
        print("  N - 输入自定义编号 (0-35)")
        print("  SPACE - 执行检测定位")
        print("  S - 保存当前结果")
        print("  Q/ESC - 退出")
        print("=" * 60)
        
        if not self.camera.start():
            print("❌ 摄像头启动失败")
            return
        
        window_name = "Screen Target Locator"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        try:
            while True:
                color_image, depth_raw, depth_colored = self.camera.get_frames()
                
                if color_image is None or depth_raw is None:
                    continue
                
                # 显示图像
                if self.last_detection_result:
                    display_image = self.visualize_result(color_image, self.last_detection_result)
                else:
                    display_image = color_image.copy()
                
                # 添加状态信息
                status_text = f"目标编号: {self.current_target_index} | 按SPACE检测"
                cv2.putText(display_image, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if self.last_torso_coords:
                    coord_text = f"Torso: {self.last_torso_coords}"
                    cv2.putText(display_image, coord_text, (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.imshow(window_name, display_image)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                
                if key in (27, ord('q')):  # ESC or Q
                    break
                    
                elif ord('0') <= key <= ord('9'):  # 数字键
                    self.current_target_index = key - ord('0')
                    print(f"\n🎯 选择目标编号: {self.current_target_index}")
                    
                elif key == ord('n'):  # 自定义输入
                    try:
                        index = int(input("请输入目标编号 (0-35): "))
                        if 0 <= index <= 35:
                            self.current_target_index = index
                            print(f"🎯 选择目标编号: {self.current_target_index}")
                        else:
                            print("❌ 编号超出范围")
                    except ValueError:
                        print("❌ 输入无效")
                    
                elif key == ord(' '):  # 空格键检测
                    print(f"\n🔍 开始检测目标区域 {self.current_target_index}...")
                    result = self.detect_and_locate(
                        color_image, depth_raw, self.current_target_index
                    )
                    if result:
                        self.last_detection_result = result
                        self.last_torso_coords = result['torso_coord']
                        print("✅ 检测成功")
                    else:
                        self.last_detection_result = None
                        self.last_torso_coords = None
                    
                elif key == ord('s'):  # 保存结果
                    if self.last_detection_result:
                        self._save_result(color_image, self.last_detection_result)
                    else:
                        print("❌ 无可保存的结果")
        
        except KeyboardInterrupt:
            print("\n⚠️  用户中断")
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
            print("[INFO] 程序已退出")
    
    def _save_result(self, color_image: np.ndarray, result: Dict[str, Any]):
        """保存检测结果"""
        output_dir = Path("data/screen_target_results")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        from datetime import datetime
        import json
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 保存可视化图像
        vis_image = self.visualize_result(color_image, result)
        img_path = output_dir / f"target_{result['target_index']}_{timestamp}.png"
        cv2.imwrite(str(img_path), vis_image)
        
        # 保存JSON数据
        json_path = output_dir / f"target_{result['target_index']}_{timestamp}.json"
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        
        print(f"\n💾 结果已保存:")
        print(f"   图像: {img_path}")
        print(f"   数据: {json_path}")


# ==========================================
# 命令行接口
# ==========================================
def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="屏幕目标定位器")
    parser.add_argument("--server", type=str, default="http://192.168.77.103:28000",
                       help="YOLO服务地址")
    parser.add_argument("--target", type=int, default=0,
                       help="目标区域编号 (0-35)")
    parser.add_argument("--mode", type=str, default="interactive",
                       choices=["interactive", "single"],
                       help="运行模式: interactive(交互式) 或 single(单次检测)")
    
    args = parser.parse_args()
    
    locator = ScreenTargetLocator(args.server)
    
    if args.mode == "interactive":
        locator.current_target_index = args.target
        locator.run_interactive()
    else:
        # 单次检测模式
        if not locator.camera.start():
            print("❌ 摄像头启动失败")
            return
        
        try:
            print("⏳ 等待稳定帧...")
            import time
            time.sleep(2)
            
            color_image, depth_raw, _ = locator.camera.get_frames()
            
            if color_image is not None and depth_raw is not None:
                result = locator.detect_and_locate(color_image, depth_raw, args.target)
                
                if result:
                    print("\n✅ 检测成功:")
                    print(f"   目标编号: {result['target_index']}")
                    print(f"   像素坐标: {result['pixel_coord']}")
                    print(f"   Torso坐标: {result['torso_coord']}")
                    
                    # 保存结果
                    locator._save_result(color_image, result)
                else:
                    print("❌ 检测失败")
            else:
                print("❌ 无法获取图像")
                
        finally:
            locator.camera.stop()


if __name__ == "__main__":
    main()