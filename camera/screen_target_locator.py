#!/usr/bin/env python3
"""
screen_target_locator.py
========================

基于YOLO检测的屏幕目标定位系统

功能:
1. 调用外部YOLO服务识别屏幕区域
2. 根据指定编号获取目标区域中心点
3. 将像素坐标转换为机器人Torso坐标系 (🆕 支持Torso Z验证)
4. 支持实时预览和交互式操作

单一检测命令行示例:
python screen_target_locator.py --mode single --target 17
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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'loco', 'unitree_sdk_python'))
from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera


# ==========================================
# 坐标转换器
# ==========================================
class CoordTransformer:
    """相机坐标系 -> Torso坐标系转换器"""
    
    def __init__(self):
        self.urdf_trans = np.array([0.0576235, 0.01753, 0.42987])
        self.pitch_offset = 0.23
        self.base_pitch = 0.8307767239493009
        self._recalc_matrices()
    
    def _recalc_matrices(self):
        final_pitch = self.base_pitch + self.pitch_offset
        self.mat_opt_to_link = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        self.urdf_rpy = [0, final_pitch, 0]
        r_obj = R.from_euler('xyz', self.urdf_rpy, degrees=False)
        try:
            self.mat_link_to_torso = r_obj.as_matrix()
        except:
            self.mat_link_to_torso = r_obj.as_dcm()
    
    def process(self, point_cam_optical: np.ndarray) -> np.ndarray:
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
        try:
            _, img_encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            files = {'file': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')}
            data = {'target_index': target_index}
            
            response = requests.post(self.endpoint, files=files, data=data, timeout=5.0)
            
            if response.status_code == 200:
                return response.json()
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
# 🆕 升级版深度辅助工具
# ==========================================
class DepthHelper:
    """深度图辅助工具 (支持Torso Z验证)"""
    
    def __init__(self, 
                 coord_transformer: CoordTransformer,
                 camera_intrinsics,
                 depth_scale: float,
                 expected_torso_z: float = -0.17,
                 torso_z_tolerance: float = 0.05):
        """
        Args:
            coord_transformer: 坐标转换器
            camera_intrinsics: 相机内参
            depth_scale: 深度比例
            expected_torso_z: 预期Torso Z基准值 (米)
            torso_z_tolerance: Z值容差 (米)
        """
        self.transformer = coord_transformer
        self.intrinsics = camera_intrinsics
        self.depth_scale = depth_scale
        self.expected_torso_z = expected_torso_z
        self.torso_z_tolerance = torso_z_tolerance
    
    def _is_torso_z_reasonable(self, torso_z: float) -> bool:
        """检查Torso Z值是否合理"""
        deviation = abs(torso_z - self.expected_torso_z)
        return deviation <= self.torso_z_tolerance
    
    def get_precise_depth_basic(self, depth_image: np.ndarray, x: int, y: int, 
                               max_search_radius: int = 20) -> Tuple[float, Tuple[int, int]]:
        """
        基础同心圆搜索 (无Torso Z验证)
        
        Returns:
            (depth_value, (offset_x, offset_y))
        """
        height, width = depth_image.shape
        
        if not (0 <= x < width and 0 <= y < height):
            return 0, (0, 0)
        
        # 策略1: 中心点
        center_depth = depth_image[y, x]
        if center_depth > 0:
            return center_depth, (0, 0)
        
        # 策略2: 同心圆搜索
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
        
        return 0, (0, 0)
    
    def collect_valid_depth_candidates(self, depth_image: np.ndarray, x: int, y: int, 
                                      max_radius: int = 50) -> list:
        """
        🆕 收集周围通过Torso Z验证的深度点
        
        Returns:
            list: [(depth_m, u, v), ...]
        """
        height, width = depth_image.shape
        valid_candidates = []
        
        for radius in range(1, max_radius + 1):
            num_samples = max(16, radius * 3)
            
            for i in range(num_samples):
                angle = 2 * np.pi * i / num_samples
                dx = int(radius * np.cos(angle))
                dy = int(radius * np.sin(angle))
                nx, ny = x + dx, y + dy
                
                if 0 <= nx < width and 0 <= ny < height:
                    depth_raw = depth_image[ny, nx]
                    if depth_raw > 0:
                        depth_m = depth_raw * self.depth_scale
                        
                        # 🆕 验证Torso Z
                        pt_cam = rs.rs2_deproject_pixel_to_point(
                            self.intrinsics, [nx, ny], depth_m
                        )
                        pt_torso = self.transformer.process(pt_cam)
                        
                        if self._is_torso_z_reasonable(pt_torso[2]):
                            valid_candidates.append((depth_m, nx, ny))
            
            if len(valid_candidates) >= 8:
                break
        
        return valid_candidates
    
    def get_depth_with_validation(self, depth_image: np.ndarray, x: int, y: int, 
                                  initial_radius: int = 20, 
                                  max_radius: int = 50) -> Optional[Dict[str, Any]]:
        """
        🆕 带Torso Z验证的深度获取
        
        流程:
        1. 常规搜索 → Torso Z验证
        2. 失败则收集正常深度点 → 取中值
        
        Returns:
            Dict包含:
                - depth_meters: float
                - actual_pixel: (x, y)
                - torso_coord: [x, y, z]
                - search_offset: (dx, dy)
                - method: 'direct' 或 'median_fill'
        """
        height, width = depth_image.shape
        
        if not (0 <= x < width and 0 <= y < height):
            return None
        
        # ========== 阶段1: 常规搜索 ==========
        depth_value, search_offset = self.get_precise_depth_basic(
            depth_image, x, y, max_search_radius=initial_radius
        )
        
        if depth_value > 0:
            dist = depth_value * self.depth_scale
            actual_u = x + search_offset[0]
            actual_v = y + search_offset[1]
            
            pt_cam = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [actual_u, actual_v], dist
            )
            pt_torso = self.transformer.process(pt_cam)
            
            if self._is_torso_z_reasonable(pt_torso[2]):
                return {
                    'depth_meters': dist,
                    'actual_pixel': (actual_u, actual_v),
                    'torso_coord': pt_torso,
                    'search_offset': search_offset,
                    'method': 'direct',
                    'torso_z_deviation': abs(pt_torso[2] - self.expected_torso_z)
                }
            else:
                print(f"  ⚠️  常规深度异常 (Torso Z={pt_torso[2]:.3f}m),扩大搜索...")
        
        # ========== 阶段2: 中值填补 ==========
        print(f"  → 收集周围正常深度点 (半径≤{max_radius}px)...")
        valid_candidates = self.collect_valid_depth_candidates(
            depth_image, x, y, max_radius=max_radius
        )
        
        if len(valid_candidates) < 3:
            print(f"  ❌ 正常深度点不足 ({len(valid_candidates)} < 3)")
            return None
        
        # 取中值深度
        depths = [c[0] for c in valid_candidates]
        median_depth = np.median(depths)
        
        print(f"  ✅ 找到 {len(valid_candidates)} 个正常点,中值深度: {median_depth:.3f}m")
        
        # 使用目标点像素 + 中值深度
        pt_cam = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [x, y], median_depth
        )
        pt_torso = self.transformer.process(pt_cam)
        
        return {
            'depth_meters': median_depth,
            'actual_pixel': (x, y),
            'torso_coord': pt_torso,
            'search_offset': (0, 0),
            'method': 'median_fill',
            'num_valid_points': len(valid_candidates),
            'torso_z_deviation': abs(pt_torso[2] - self.expected_torso_z)
        }


# ==========================================
# 主应用类
# ==========================================
class ScreenTargetLocator:
    """屏幕目标定位器"""
    
    def __init__(self, 
                 yolo_server_url: str = "http://192.168.77.103:28000",
                 expected_torso_z: float = -0.17,
                 torso_z_tolerance: float = 0.05):
        """
        Args:
            yolo_server_url: YOLO服务地址
            expected_torso_z: 屏幕平面的Torso Z基准值 (米)
            torso_z_tolerance: Z值容差 (米)
        """
        # 组件初始化
        self.camera = RealSenseCamera(width=848, height=480, fps=30)
        self.yolo_client = YOLOClient(yolo_server_url)
        self.coord_transformer = CoordTransformer()
        
        # 🆕 等待相机启动后再初始化 DepthHelper
        self.depth_helper = None
        self.expected_torso_z = expected_torso_z
        self.torso_z_tolerance = torso_z_tolerance
        
        # 状态变量
        self.current_target_index = 0
        self.last_detection_result = None
        self.last_torso_coords = None
        
        print(f"✅ 屏幕目标定位器初始化完成")
        print(f"   YOLO服务: {yolo_server_url}")
        print(f"   摄像头分辨率: 848x480")
        print(f"   🆕 Torso Z基准: {expected_torso_z:.3f}m (±{torso_z_tolerance*100:.0f}cm)")
    
    def _init_depth_helper(self):
        """初始化深度辅助工具 (需要相机已启动)"""
        if self.depth_helper is None:
            self.depth_helper = DepthHelper(
                coord_transformer=self.coord_transformer,
                camera_intrinsics=self.camera.depth_intrinsics,
                depth_scale=self.camera.depth_scale,
                expected_torso_z=self.expected_torso_z,
                torso_z_tolerance=self.torso_z_tolerance
            )
    
    def detect_and_locate(self, color_image: np.ndarray, depth_raw: np.ndarray, 
                         target_index: int) -> Optional[Dict[str, Any]]:
        """
        检测屏幕并定位目标区域的Torso坐标
        
        Returns:
            Dict包含:
                - target_index: int
                - pixel_coord: (x, y)
                - depth_meters: float
                - camera_coord: [x, y, z]
                - torso_coord: [x, y, z]
                - 🆕 method: 'direct' 或 'median_fill'
                - 🆕 torso_z_deviation: float
        """
        # 1. 调用YOLO服务
        yolo_result = self.yolo_client.detect_screen_target(color_image, target_index)
        
        if not yolo_result or not yolo_result.get('found'):
            print(f"❌ 未检测到屏幕或目标区域")
            return None
        
        # 2. 提取目标中心点
        target_center = yolo_result['target_region']['center']
        pixel_x, pixel_y = target_center
        
        print(f"\n📍 目标区域 {target_index} 中心: ({pixel_x}, {pixel_y})")
        
        # 3. 🆕 使用升级版深度获取
        depth_result = self.depth_helper.get_depth_with_validation(
            depth_raw, pixel_x, pixel_y,
            initial_radius=20, max_radius=50
        )
        
        if depth_result is None:
            print(f"❌ 无法获取有效深度值")
            return None
        
        # 4. 提取结果
        depth_meters = depth_result['depth_meters']
        actual_pixel = depth_result['actual_pixel']
        torso_point = depth_result['torso_coord']
        method = depth_result['method']
        
        print(f"📏 深度: {depth_meters:.3f}m (方法: {method})")
        
        if method == 'median_fill':
            print(f"   基于 {depth_result['num_valid_points']} 个正常点的中值")
        
        # 5. 计算相机坐标
        camera_point = rs.rs2_deproject_pixel_to_point(
            self.camera.depth_intrinsics,
            list(actual_pixel),
            depth_meters
        )
        
        print(f"📷 相机坐标: X={camera_point[0]:.3f}, Y={camera_point[1]:.3f}, Z={camera_point[2]:.3f}")
        print(f"🤖 Torso坐标: X={torso_point[0]:.3f}, Y={torso_point[1]:.3f}, Z={torso_point[2]:.3f}")
        print(f"📊 Torso Z偏差: {depth_result['torso_z_deviation']*100:.1f}cm")
        
        return {
            'target_index': target_index,
            'pixel_coord': (pixel_x, pixel_y),
            'actual_pixel_coord': actual_pixel,
            'depth_meters': float(depth_meters),
            'camera_coord': list(camera_point),
            'torso_coord': torso_point.tolist(),
            'screen_corners': yolo_result['screen_corners'],
            'target_region': yolo_result['target_region'],
            'search_offset': depth_result['search_offset'],
            'method': method,  # 🆕
            'torso_z_deviation': depth_result['torso_z_deviation']  # 🆕
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
        
        # 🆕 根据方法选择颜色
        method = result.get('method', 'direct')
        if method == 'median_fill':
            marker_color = (255, 165, 0)  # 橙色
            marker_type = cv2.MARKER_DIAMOND
        else:
            marker_color = (0, 255, 0)  # 绿色
            marker_type = cv2.MARKER_TILTED_CROSS
        
        cv2.drawMarker(vis_image, center, (0, 255, 255), 
                      cv2.MARKER_TILTED_CROSS, 15, 2)
        
        if result['search_offset'] != (0, 0):
            cv2.circle(vis_image, actual_center, 5, (0, 0, 255), -1)
            cv2.line(vis_image, center, actual_center, (255, 255, 0), 1)
        else:
            cv2.drawMarker(vis_image, center, marker_color, marker_type, 10, 2)
        
        # 显示Torso坐标
        torso = result['torso_coord']
        z_dev = result.get('torso_z_deviation', 0) * 100
        label = f"Grid{result['target_index']}: Z={torso[2]:.2f}m (±{z_dev:.0f}cm)"
        cv2.putText(vis_image, label, (actual_center[0]+12, actual_center[1]-5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, marker_color, 2)
        
        return vis_image
    
    def run_interactive(self):
        """运行交互式定位模式"""
        print("\n🚀 启动屏幕目标定位器")
        print("\n操作说明:")
        print("  0-9 - 快速选择目标编号 (0-9)")
        print("  N - 输入自定义编号 (0-35)")
        print("  SPACE - 执行检测定位")
        print("  S - 保存当前结果")
        print("  +/- - 调整Z容差")
        print("  [/] - 微调Z基准")
        print("  Q/ESC - 退出")
        print("=" * 60)
        
        if not self.camera.start():
            print("❌ 摄像头启动失败")
            return
        
        # 🆕 初始化DepthHelper
        self._init_depth_helper()
        
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
                status_text = f"Target:{self.current_target_index} | Z:{self.expected_torso_z:.2f}m(±{self.torso_z_tolerance*100:.0f}cm)"
                cv2.putText(display_image, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if self.last_torso_coords:
                    coord_text = f"Torso XYZ: {[f'{c:.2f}' for c in self.last_torso_coords]}"
                    cv2.putText(display_image, coord_text, (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.imshow(window_name, display_image)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                
                if key in (27, ord('q')):
                    break
                elif ord('0') <= key <= ord('9'):
                    self.current_target_index = key - ord('0')
                    print(f"\n🎯 选择目标编号: {self.current_target_index}")
                elif key == ord('n'):
                    try:
                        index = int(input("请输入目标编号 (0-35): "))
                        if 0 <= index <= 35:
                            self.current_target_index = index
                            print(f"🎯 选择目标编号: {self.current_target_index}")
                        else:
                            print("❌ 编号超出范围")
                    except ValueError:
                        print("❌ 输入无效")
                elif key == ord(' '):
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
                elif key == ord('s'):
                    if self.last_detection_result:
                        self._save_result(color_image, self.last_detection_result)
                    else:
                        print("❌ 无可保存的结果")
                # 🆕 新增快捷键
                elif key == ord('+') or key == ord('='):
                    self.torso_z_tolerance += 0.01
                    self.depth_helper.torso_z_tolerance = self.torso_z_tolerance
                    print(f"📏 Z容差: ±{self.torso_z_tolerance*100:.0f}cm")
                elif key == ord('-') or key == ord('_'):
                    self.torso_z_tolerance = max(0.01, self.torso_z_tolerance - 0.01)
                    self.depth_helper.torso_z_tolerance = self.torso_z_tolerance
                    print(f"📏 Z容差: ±{self.torso_z_tolerance*100:.0f}cm")
                elif key == ord('['):
                    self.expected_torso_z -= 0.01
                    self.depth_helper.expected_torso_z = self.expected_torso_z
                    print(f"📏 Z基准: {self.expected_torso_z:.3f}m")
                elif key == ord(']'):
                    self.expected_torso_z += 0.01
                    self.depth_helper.expected_torso_z = self.expected_torso_z
                    print(f"📏 Z基准: {self.expected_torso_z:.3f}m")
        
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
        
        vis_image = self.visualize_result(color_image, result)
        img_path = output_dir / f"target_{result['target_index']}_{timestamp}.png"
        cv2.imwrite(str(img_path), vis_image)
        
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
    parser.add_argument("--torso-z", type=float, default=-0.17,
                       help="屏幕Torso Z基准值 (米)")
    parser.add_argument("--z-tolerance", type=float, default=0.05,
                       help="Z值容差 (米)")
    
    args = parser.parse_args()
    
    locator = ScreenTargetLocator(
        args.server,
        expected_torso_z=args.torso_z,
        torso_z_tolerance=args.z_tolerance
    )
    
    if args.mode == "interactive":
        locator.current_target_index = args.target
        locator.run_interactive()
    else:
        # 单次检测模式
        if not locator.camera.start():
            print("❌ 摄像头启动失败")
            return
        
        locator._init_depth_helper()
        
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
                    print(f"   方法: {result['method']}")
                    print(f"   Z偏差: {result['torso_z_deviation']*100:.1f}cm")
                    
                    locator._save_result(color_image, result)
                else:
                    print("❌ 检测失败")
            else:
                print("❌ 无法获取图像")
                
        finally:
            locator.camera.stop()


if __name__ == "__main__":
    main()