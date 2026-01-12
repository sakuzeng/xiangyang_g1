import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class CoordTransfomer:
    def __init__(self):
        self.urdf_trans = np.array([0.0576235, 0.01753, 0.42987]) 
        
        # ==========================================
        # 🔴 [在此处修改校准参数]
        # ==========================================
        # 现象：地面呈现"上坡" (远处 Z > 近处 Z)
        # 目标：需要让相机"低头"更多，把远处的地面压下去
        # 动作：增加正向 Pitch
        
        self.pitch_offset = 0.15  # 尝试增加 0.15 (约8.5度)
        
        # 基础值 (URDF)
        self.base_pitch = 0.8307767239493009
        
        # 立即重新计算矩阵
        self._recalc_matrices()

    def _recalc_matrices(self):
        """重新计算旋转矩阵，并打印调试信息"""
        final_pitch = self.base_pitch + self.pitch_offset
        self.urdf_rpy = [0, final_pitch, 0]
        
        print("\n" + "="*40)
        print(f"[DEBUG] 矩阵重算生效确认:")
        print(f"  > 原始 Pitch: {self.base_pitch:.4f}")
        print(f"  > 偏移 Offset: {self.pitch_offset:.4f}")
        print(f"  > 最终 Pitch: {final_pitch:.4f} rad ({np.degrees(final_pitch):.1f} 度)")
        print("="*40 + "\n")

        # 1. Optical -> Link
        self.mat_opt_to_link = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ])

        # 2. Link -> Torso
        r_obj = R.from_euler('xyz', self.urdf_rpy, degrees=False)
        try:
            self.mat_link_to_torso = r_obj.as_matrix()
        except AttributeError:
            self.mat_link_to_torso = r_obj.as_dcm()

    def process(self, point_cam_optical):
        P_opt = np.array(point_cam_optical)
        P_link = self.mat_opt_to_link @ P_opt
        P_torso = self.mat_link_to_torso @ P_link + self.urdf_trans
        return P_torso

class CameraApp:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        self.align = rs.align(rs.stream.color)
        
        # 实例化转换器 (会触发打印)
        self.transformer = CoordTransfomer()
        
        self.hole_filling = rs.hole_filling_filter(1) 
        self.mouse_pos = (-1, -1)
        self.click_flag = False

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_pos = (x, y)
            self.click_flag = True

    def get_robust_depth(self, depth_image, x, y, radius=2):
        height, width = depth_image.shape
        if 0 <= x < width and 0 <= y < height:
            d = depth_image[y, x]
            if d > 0: return d
            valid_pixels = []
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= nx < width and 0 <= ny < height:
                        val = depth_image[ny, nx]
                        if val > 0: valid_pixels.append(val)
            if valid_pixels: return np.median(valid_pixels)
        return 0

    def run(self):
        print("[INFO] 正在启动相机...")
        profile = self.pipeline.start(self.config)
        depth_stream = profile.get_stream(rs.stream.depth)
        intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        cv2.namedWindow("Calibrate")
        cv2.setMouseCallback("Calibrate", self.mouse_callback)

        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)
                depth_frame = self.hole_filling.process(frames.get_depth_frame())
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame: continue

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                if self.click_flag:
                    self.click_flag = False
                    u, v = self.mouse_pos
                    dist = self.get_robust_depth(depth_image, u, v, radius=4) * depth_scale
                    
                    if dist > 0:
                        pt_opt = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], dist)
                        pt_torso = self.transformer.process(pt_opt)
                        
                        print(f"📍 ({u}, {v}) D={dist:.3f}m -> Torso Z: {pt_torso[2]:.3f}m | X: {pt_torso[0]:.3f}m")
                        
                        label = f"Z: {pt_torso[2]:.3f}"
                        cv2.circle(color_image, (u, v), 5, (0, 0, 255), -1)
                        cv2.putText(color_image, label, (u+10, v), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)

                cv2.imshow("Calibrate", color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    CameraApp().run()