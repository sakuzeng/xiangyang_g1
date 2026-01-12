import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'loco', 'unitree_sdk_python'))
from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera

class CoordTransfomer:
    def __init__(self):
        self.urdf_trans = np.array([0.0576235, 0.01753, 0.42987]) 
        self.pitch_offset = 0.23 
        self.base_pitch = 0.8307767239493009
        self._recalc_matrices()

    def _recalc_matrices(self):
        final_pitch = self.base_pitch + self.pitch_offset
        self.mat_opt_to_link = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        self.urdf_rpy = [0, final_pitch, 0]
        r_obj = R.from_euler('xyz', self.urdf_rpy, degrees=False)
        try: 
            self.mat_link_to_torso = r_obj.as_matrix()
        except: 
            self.mat_link_to_torso = r_obj.as_dcm()

    def process(self, point_cam_optical):
        P_opt = np.array(point_cam_optical)
        P_link = self.mat_opt_to_link @ P_opt
        P_torso = self.mat_link_to_torso @ P_link + self.urdf_trans
        return P_torso


class CameraApp:
    def __init__(self, 
                 default_torso_z: float = -0.17,
                 use_default_z: bool = True):
        """
        Args:
            default_torso_z: 默认Torso Z基准值(米)
            use_default_z: 是否使用默认值(True=跳过初始化)
        """
        self.camera = RealSenseCamera(width=848, height=480, fps=30)
        self.transformer = CoordTransfomer()
        
        self.image_width = 848
        self.mouse_pos = (-1, -1)
        self.click_pos = None
        self.click_flag = False
        self.last_result = None
        
        # Torso Z值约束
        self.torso_z_history = []
        self.max_history = 10
        
        if use_default_z:
            self.expected_torso_z = default_torso_z
            print(f"  ✅ 使用预设Torso Z基准: {self.expected_torso_z:.3f}m")
        else:
            self.expected_torso_z = None
            print(f"  ⏳ 等待自动学习Z基准 (需5个点)...")
        
        self.torso_z_tolerance = 0.05  # ±5cm

    def mouse_callback(self, event, x, y, flags, param):
        if x < self.image_width:
            self.mouse_pos = (x, y)
        else:
            self.mouse_pos = (x - self.image_width, y)
        
        if event == cv2.EVENT_LBUTTONDOWN:
            if x < self.image_width:
                self.click_pos = (x, y)
            else:
                self.click_pos = (x - self.image_width, y)
            self.click_flag = True

    def _is_torso_z_reasonable(self, torso_z: float) -> bool:
        """检查Torso Z值是否合理"""
        if self.expected_torso_z is not None:
            deviation = abs(torso_z - self.expected_torso_z)
            if deviation > self.torso_z_tolerance:
                return False
        elif len(self.torso_z_history) >= 3:
            median_z = np.median(self.torso_z_history)
            if abs(torso_z - median_z) > 0.10:
                return False
        return True

    def _update_torso_z_reference(self):
        """更新Torso Z基准值"""
        if len(self.torso_z_history) >= 5 and self.expected_torso_z is None:
            self.expected_torso_z = np.median(self.torso_z_history)
            print(f"  ✅ 自动建立Torso Z基准: {self.expected_torso_z:.3f}m (±{self.torso_z_tolerance*100:.0f}cm)")

    def collect_valid_depth_candidates(self, depth_image, x, y, intrinsics, max_radius=50):
        """
        🟢 收集周围有效深度候选点 (基于Torso Z验证)
        
        Args:
            depth_image: 深度图
            x, y: 目标像素
            intrinsics: 相机内参
            max_radius: 最大搜索半径
        
        Returns:
            list: [(depth_m1, u1, v1), ...] 通过Torso Z验证的深度点
        """
        height, width = depth_image.shape
        depth_scale = self.camera.depth_scale
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
                        depth_m = depth_raw * depth_scale
                        
                        # 🆕 关键: 验证该深度对应的Torso Z是否合理
                        pt_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [nx, ny], depth_m)
                        pt_torso = self.transformer.process(pt_cam)
                        
                        if self._is_torso_z_reasonable(pt_torso[2]):
                            valid_candidates.append((depth_m, nx, ny))
            
            # 如果已找到足够多的有效点,提前退出
            if len(valid_candidates) >= 8:
                break
        
        return valid_candidates

    def get_depth_with_validation(self, depth_image, x, y, intrinsics, 
                                   initial_radius=20, max_radius=50):
        """
        🟢 简化版: 常规搜索 → Torso Z验证 → 取正常深度中值
        """
        height, width = depth_image.shape
        depth_scale = self.camera.depth_scale
        
        if not (0 <= x < width and 0 <= y < height):
            return None
        
        # ========== 阶段1: 常规搜索 ==========
        depth_value, search_offset = self.get_precise_depth(
            depth_image, x, y, max_search_radius=initial_radius
        )
        
        if depth_value > 0:
            dist = depth_value * depth_scale
            actual_u = x + search_offset[0]
            actual_v = y + search_offset[1]
            
            pt_opt = rs.rs2_deproject_pixel_to_point(intrinsics, [actual_u, actual_v], dist)
            pt_torso = self.transformer.process(pt_opt)
            
            if self._is_torso_z_reasonable(pt_torso[2]):
                print(f"  ✅ 常规搜索通过 (offset={search_offset})")
                return {
                    'depth_meters': dist,
                    'actual_pixel': (actual_u, actual_v),
                    'torso_coord': pt_torso,
                    'search_offset': search_offset,
                    'method': 'direct'
                }
            else:
                print(f"  ⚠️  常规深度异常 (Z={pt_torso[2]:.3f}m),扩大搜索...")
        
        # ========== 阶段2: 🆕 收集正常深度点并取中值 ==========
        print(f"  → 收集周围正常深度点 (半径≤{max_radius}px)...")
        valid_candidates = self.collect_valid_depth_candidates(
            depth_image, x, y, intrinsics, max_radius=max_radius
        )
        
        if len(valid_candidates) < 3:
            print(f"  ❌ 正常深度点不足 ({len(valid_candidates)} < 3)")
            return None
        
        # 🆕 取中值深度
        depths = [c[0] for c in valid_candidates]
        median_depth = np.median(depths)
        
        # 找到最接近中值的点
        best_idx = np.argmin([abs(d - median_depth) for d in depths])
        best_depth, best_u, best_v = valid_candidates[best_idx]
        
        print(f"  ✅ 找到 {len(valid_candidates)} 个正常点,中值深度: {median_depth:.3f}m")
        print(f"     使用点 ({best_u}, {best_v}) 深度: {best_depth:.3f}m")
        
        # 🆕 使用目标点像素 + 中值深度重新计算
        pt_opt = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], median_depth)
        pt_torso = self.transformer.process(pt_opt)
        
        return {
            'depth_meters': median_depth,
            'actual_pixel': (x, y),
            'torso_coord': pt_torso,
            'search_offset': (0, 0),
            'method': 'median_fill',
            'num_valid_points': len(valid_candidates)
        }

    def get_precise_depth(self, depth_image, x, y, max_search_radius=20):
        """同心圆搜索 (原逻辑)"""
        height, width = depth_image.shape
        
        if not (0 <= x < width and 0 <= y < height):
            return 0, (0, 0)
        
        center_depth = depth_image[y, x]
        if center_depth > 0:
            return center_depth, (0, 0)
        
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
                print(f"  → 搜索半径 {r}px: depth={depth_val}")
                return depth_val, (dx, dy)
        
        return 0, (0, 0)

    def run(self):
        print("[INFO] 正在启动相机...")
        
        if not self.camera.start():
            print("❌ 相机启动失败")
            return
        
        intrinsics = self.camera.depth_intrinsics

        cv2.namedWindow("RealSense High Precision")
        cv2.setMouseCallback("RealSense High Precision", self.mouse_callback)

        print("\n=== 系统就绪 (中值填补模式) ===")
        print("📌 左侧: 彩色图 | 右侧: 深度图")
        print("🎯 策略: 常规搜索 → Torso Z验证 → 正常点中值填补")
        if self.expected_torso_z is not None:
            print(f"📏 Z基准: {self.expected_torso_z:.3f}m (±{self.torso_z_tolerance*100:.0f}cm)")
        print("\n⌨️  快捷键: Q:退出 | S:保存 | C:清除 | R:重置 | Z:设置基准 | +/-:调容差 | [/]:调基准")

        try:
            while True:
                color_image, depth_raw, depth_colored = self.camera.get_frames()
                
                if color_image is None or depth_raw is None:
                    continue

                display_color = color_image.copy()
                display_depth = depth_colored.copy()

                if self.click_flag:
                    self.click_flag = False
                    u, v = self.click_pos
                    
                    result = self.get_depth_with_validation(
                        depth_raw, u, v, intrinsics,
                        initial_radius=20, max_radius=50
                    )
                    
                    if result:
                        dist = result['depth_meters']
                        pt_torso = result['torso_coord']
                        actual_u, actual_v = result['actual_pixel']
                        search_offset = result['search_offset']
                        method = result['method']
                        
                        # 只在自动学习模式下更新历史
                        if self.expected_torso_z is None:
                            self.torso_z_history.append(pt_torso[2])
                            if len(self.torso_z_history) > self.max_history:
                                self.torso_z_history.pop(0)
                            self._update_torso_z_reference()
                        
                        self.last_result = {
                            'pixel': (u, v),
                            'actual_pixel': (actual_u, actual_v),
                            'depth': dist,
                            'torso': pt_torso,
                            'search_offset': search_offset,
                            'method': method
                        }

                        print(f"\n📍 点击: ({u}, {v})")
                        if search_offset != (0, 0):
                            print(f"🔍 实际: ({actual_u}, {actual_v}) [偏移 {search_offset}]")
                        print(f"📏 深度: {dist:.3f}m")
                        print(f"🤖 Torso: X={pt_torso[0]:.3f}, Y={pt_torso[1]:.3f}, Z={pt_torso[2]:.3f}")
                        print(f"🔧 方法: {method}")
                        
                        if method == 'median_fill':
                            print(f"   正常点数: {result['num_valid_points']}")
                        
                        if self.expected_torso_z is not None:
                            z_dev = abs(pt_torso[2] - self.expected_torso_z)
                            status = "✅" if z_dev <= self.torso_z_tolerance else "⚠️"
                            print(f"📊 {status} Z偏差: {z_dev*100:.1f}cm (基准: {self.expected_torso_z:.3f}m)")
                    else:
                        print(f"\n❌ 无法获取有效深度")
                        self.last_result = None

                # 绘制
                if self.last_result:
                    u, v = self.last_result['pixel']
                    actual_u, actual_v = self.last_result['actual_pixel']
                    pt_torso = self.last_result['torso']
                    dist = self.last_result['depth']
                    offset = self.last_result['search_offset']
                    method = self.last_result.get('method', 'unknown')
                    
                    if method == 'median_fill':
                        color = (255, 165, 0)  # 橙色
                        marker_type = cv2.MARKER_DIAMOND
                    else:
                        color = (0, 255, 0)
                        marker_type = cv2.MARKER_TILTED_CROSS
                    
                    label = f"Z:{pt_torso[2]:.2f}m D:{dist:.2f}m"
                    
                    cv2.drawMarker(display_color, (u, v), (0, 255, 255), 
                                  cv2.MARKER_TILTED_CROSS, 15, 2)
                    
                    if offset != (0, 0):
                        cv2.circle(display_color, (actual_u, actual_v), 5, (0, 0, 255), -1)
                        cv2.line(display_color, (u, v), (actual_u, actual_v), (255, 255, 0), 1)
                    else:
                        cv2.drawMarker(display_color, (u, v), color, marker_type, 10, 2)
                    
                    cv2.putText(display_color, label, (actual_u+12, actual_v-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    
                    cv2.drawMarker(display_depth, (u, v), (0, 255, 255), 
                                  cv2.MARKER_TILTED_CROSS, 15, 2)
                    if offset != (0, 0):
                        cv2.circle(display_depth, (actual_u, actual_v), 5, (0, 0, 255), -1)
                    else:
                        cv2.drawMarker(display_depth, (u, v), color, marker_type, 10, 2)
                    
                    cv2.putText(display_depth, label, (actual_u+12, actual_v-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                if self.mouse_pos[0] >= 0:
                    mx, my = self.mouse_pos
                    cv2.drawMarker(display_color, (mx, my), (0, 255, 255), 
                                  cv2.MARKER_CROSS, 12, 1)
                    cv2.drawMarker(display_depth, (mx, my), (0, 255, 255), 
                                  cv2.MARKER_CROSS, 12, 1)
                
                display_image = cv2.hconcat([display_color, display_depth])
                h = display_image.shape[0]
                cv2.line(display_image, (self.image_width, 0), 
                        (self.image_width, h), (255, 255, 255), 2)
                
                cv2.imshow("RealSense High Precision", display_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.camera.save_images("data/images", prefix="torso")
                    print("💾 图像已保存")
                elif key == ord('c'):
                    self.last_result = None
                    print("🧹 标记已清除")
                elif key == ord('r'):
                    self.torso_z_history.clear()
                    self.expected_torso_z = None
                    print("🔄 已重置")
                elif key == ord('z'):
                    if self.last_result:
                        self.expected_torso_z = self.last_result['torso'][2]
                        print(f"✅ 设置Z基准: {self.expected_torso_z:.3f}m")
                    else:
                        print("⚠️  请先点击")
                elif key == ord('+') or key == ord('='):
                    self.torso_z_tolerance += 0.01
                    print(f"📏 Z容差: ±{self.torso_z_tolerance*100:.0f}cm")
                elif key == ord('-') or key == ord('_'):
                    self.torso_z_tolerance = max(0.01, self.torso_z_tolerance - 0.01)
                    print(f"📏 Z容差: ±{self.torso_z_tolerance*100:.0f}cm")
                elif key == ord('['):
                    if self.expected_torso_z is not None:
                        self.expected_torso_z -= 0.01
                        print(f"📏 Z基准: {self.expected_torso_z:.3f}m")
                elif key == ord(']'):
                    if self.expected_torso_z is not None:
                        self.expected_torso_z += 0.01
                        print(f"📏 Z基准: {self.expected_torso_z:.3f}m")

        except KeyboardInterrupt:
            print("\n⚠️  用户中断")
        except Exception as e:
            print(f"❌ 程序异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
            print("[INFO] 程序已退出")


if __name__ == "__main__":
    app = CameraApp(
        default_torso_z=-0.17,
        use_default_z=True
    )
    app.run()