import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import sys
import os

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'loco', 'unitree_sdk_python'))

from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera

class CoordTransfomer:
    def __init__(self):
        # URDF 原始平移参数
        self.urdf_trans = np.array([0.0576235, 0.01753, 0.42987]) 
        
        # 校准好的 Pitch
        self.pitch_offset = 0.23 
        self.base_pitch = 0.8307767239493009
        
        self._recalc_matrices()

    def _recalc_matrices(self):
        final_pitch = self.base_pitch + self.pitch_offset
        
        # Optical -> Link
        self.mat_opt_to_link = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        
        # Link -> Torso
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
    def __init__(self):
        # 使用 RealSenseCamera 封装类
        self.camera = RealSenseCamera(width=848, height=480, fps=30)
        self.transformer = CoordTransfomer()
        
        # 初始化所有必要的变量
        self.image_width = 848
        self.mouse_pos = (-1, -1)
        self.click_pos = None
        self.click_flag = False
        self.last_result = None

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            actual_x = x % self.image_width
            self.mouse_pos = (actual_x, y)
            
        elif event == cv2.EVENT_LBUTTONDOWN:
            actual_x = x % self.image_width
            self.mouse_pos = (actual_x, y)
            self.click_pos = (actual_x, y)
            self.click_flag = True

    def get_precise_depth(self, depth_image, x, y, max_search_radius=20):
        """
        🟢 改进版: 平面约束下的智能深度搜索
        
        策略:
        1. 优先检查中心点
        2. 同心圆扩散搜索 (从中心向外)
        3. 返回距离最近的有效深度值
        4. 可视化搜索半径
        
        Args:
            depth_image: 深度图
            x, y: 目标像素坐标
            max_search_radius: 最大搜索半径 (像素)
        
        Returns:
            tuple: (depth_value, search_offset)
                - depth_value: 有效深度值
                - search_offset: 搜索偏移量 (x_offset, y_offset)
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
            # 生成圆周上的采样点
            candidates = []
            
            # 使用Bresenham圆算法生成均匀分布的点
            num_samples = max(8, radius * 2)  # 根据半径调整采样密度
            for i in range(num_samples):
                angle = 2 * np.pi * i / num_samples
                dx = int(radius * np.cos(angle))
                dy = int(radius * np.sin(angle))
                
                nx, ny = x + dx, y + dy
                
                # 边界检查
                if 0 <= nx < width and 0 <= ny < height:
                    depth_val = depth_image[ny, nx]
                    if depth_val > 0:
                        candidates.append((depth_val, dx, dy, radius))
            
            # 找到当前半径上的有效深度
            if candidates:
                # 🟢 优先选择深度值最接近平均值的点 (避免异常值)
                depths = [c[0] for c in candidates]
                median_depth = np.median(depths)
                
                # 选择最接近中值的点
                best_candidate = min(candidates, 
                                   key=lambda c: abs(c[0] - median_depth))
                
                depth_val, dx, dy, r = best_candidate
                print(f"  → 搜索半径 {r}px 处找到有效深度: {depth_val}")
                return depth_val, (dx, dy)
        
        # 策略3: 如果仍未找到,使用区域中值填补
        print(f"  → 未找到有效深度,尝试区域填补...")
        x_min = max(0, x - max_search_radius)
        x_max = min(width, x + max_search_radius + 1)
        y_min = max(0, y - max_search_radius)
        y_max = min(height, y + max_search_radius + 1)
        
        roi = depth_image[y_min:y_max, x_min:x_max]
        valid_pixels = roi[roi > 0]
        
        if len(valid_pixels) > 10:
            median_val = np.median(valid_pixels)
            print(f"  → 使用区域中值: {median_val}")
            return median_val, (0, 0)
        
        return 0, (0, 0)

    def run(self):
        print("[INFO] 正在启动相机...")
        
        if not self.camera.start():
            print("❌ 相机启动失败")
            return
        
        intrinsics = self.camera.depth_intrinsics
        depth_scale = self.camera.depth_scale

        cv2.namedWindow("RealSense High Precision")
        cv2.setMouseCallback("RealSense High Precision", self.mouse_callback)

        print("\n=== 系统就绪 (平面深度搜索模式) ===")
        print("📌 左侧: 彩色图 | 右侧: 深度图")
        print("🔍 自动搜索最近有效深度 (最大半径20px)")
        print("⌨️  按 'Q' 退出 | 按 'S' 保存 | 按 'C' 清除")

        try:
            while True:
                color_image, depth_raw, depth_colored = self.camera.get_frames()
                
                if color_image is None or depth_raw is None:
                    continue

                display_color = color_image.copy()
                display_depth = depth_colored.copy()

                # 处理点击事件
                if self.click_flag:
                    self.click_flag = False
                    u, v = self.click_pos
                    
                    # 🟢 使用改进的深度搜索
                    depth_value, search_offset = self.get_precise_depth(
                        depth_raw, u, v, max_search_radius=20
                    )
                    dist = depth_value * depth_scale
                    
                    if dist > 0:
                        # 计算实际使用的像素坐标
                        actual_u = u + search_offset[0]
                        actual_v = v + search_offset[1]
                        
                        # 反投影到相机坐标系
                        pt_opt = rs.rs2_deproject_pixel_to_point(
                            intrinsics, [actual_u, actual_v], dist
                        )
                        pt_torso = self.transformer.process(pt_opt)

                        self.last_result = {
                            'pixel': (u, v),
                            'actual_pixel': (actual_u, actual_v),
                            'depth': dist,
                            'torso': pt_torso,
                            'search_offset': search_offset
                        }

                        print(f"\n📍 点击: ({u}, {v})")
                        if search_offset != (0, 0):
                            print(f"🔍 实际: ({actual_u}, {actual_v}) [偏移 {search_offset}]")
                        print(f"📏 深度: {dist:.3f}m")
                        print(f"🤖 Torso: X={pt_torso[0]:.3f}, Y={pt_torso[1]:.3f}, Z={pt_torso[2]:.3f}")
                    else:
                        print(f"\n❌ 像素: ({u}, {v}) - 搜索半径20px内无有效深度")
                        self.last_result = None

                # 🟢 绘制增强版标记
                if self.last_result:
                    u, v = self.last_result['pixel']
                    actual_u, actual_v = self.last_result['actual_pixel']
                    pt_torso = self.last_result['torso']
                    dist = self.last_result['depth']
                    offset = self.last_result['search_offset']
                    
                    label = f"Z:{pt_torso[2]:.2f}m D:{dist:.2f}m"
                    
                    # 在彩色图上绘制
                    # 点击位置 (黄色叉)
                    cv2.drawMarker(display_color, (u, v), (0, 255, 255), 
                                  cv2.MARKER_TILTED_CROSS, 15, 2)
                    
                    # 实际采样位置 (红色圆)
                    if offset != (0, 0):
                        cv2.circle(display_color, (actual_u, actual_v), 5, (0, 0, 255), -1)
                        cv2.circle(display_color, (actual_u, actual_v), 8, (0, 255, 255), 1)
                        # 连接线
                        cv2.line(display_color, (u, v), (actual_u, actual_v), 
                                (255, 255, 0), 1)
                    else:
                        cv2.circle(display_color, (u, v), 5, (0, 255, 0), -1)
                    
                    cv2.putText(display_color, label, (actual_u+12, actual_v-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # 在深度图上同样绘制
                    cv2.drawMarker(display_depth, (u, v), (0, 255, 255), 
                                  cv2.MARKER_TILTED_CROSS, 15, 2)
                    if offset != (0, 0):
                        cv2.circle(display_depth, (actual_u, actual_v), 5, (0, 0, 255), -1)
                        cv2.line(display_depth, (u, v), (actual_u, actual_v), 
                                (255, 255, 0), 1)
                    else:
                        cv2.circle(display_depth, (u, v), 5, (0, 255, 0), -1)
                    
                    cv2.putText(display_depth, label, (actual_u+12, actual_v-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 鼠标悬停标记
                if self.mouse_pos[0] >= 0:
                    mx, my = self.mouse_pos
                    cv2.drawMarker(display_color, (mx, my), (0, 255, 255), 
                                  cv2.MARKER_CROSS, 12, 1)
                    cv2.drawMarker(display_depth, (mx, my), (0, 255, 255), 
                                  cv2.MARKER_CROSS, 12, 1)
                
                # 并排显示
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
    CameraApp().run()