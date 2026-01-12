#!/usr/bin/env python3
"""
RealSense Camera Client
=======================

通用 RealSense 摄像头接口,支持彩色流和深度流捕获。

功能特性:
- 支持多种分辨率配置 (1920x1080, 1280x720, 960x540, 640x480)
- 自动深度流分辨率匹配
- 后台线程持续捕获,线程安全的图像获取
- 深度后处理滤波 (空间滤波 + 时间滤波)
- 设备占用检测和自动重置
- 深度尺度自动获取
- 支持图像保存功能

使用示例:
    from unitree_sdk2py.camera.realsense_camera_client import RealSenseCamera
    
    camera = RealSenseCamera(width=1280, height=720)
    if camera.start():
        rgb, depth_raw, depth_colored = camera.get_frames()
        camera.stop()

作者: [Your Name]
日期: 2024
"""

from __future__ import annotations

import sys
import time
import subprocess
import os
import grp
from typing import Optional, Tuple, Dict
from datetime import datetime
import threading

try:
    import pyrealsense2 as rs
    import numpy as np
    import cv2
except ImportError as exc:
    raise SystemExit(
        "依赖缺失。请安装: pip install pyrealsense2 opencv-python numpy"
    ) from exc


# ============================================================================
# 设备管理辅助函数
# ============================================================================

def check_device_availability() -> bool:
    """
    检查 RealSense 设备是否被其他进程占用
    
    Returns:
        bool: True 表示设备可用
    """
    try:
        result = subprocess.run(
            ['lsof', '/dev/video*'], 
            capture_output=True, 
            text=True, 
            check=False
        )
        if result.stdout:
            print("[RealSense] 警告: 检测到摄像头设备被占用:")
            print(result.stdout)
            return False
        return True
    except FileNotFoundError:
        print("[RealSense] lsof 命令不可用,跳过设备检查")
        return True
    except Exception as e:
        print(f"[RealSense] 设备检查失败: {e}")
        return True


def check_video_permissions() -> bool:
    """
    检查当前用户是否有访问视频设备的权限
    
    Returns:
        bool: True 表示有权限
    """
    if os.name != 'posix':
        return True
    
    try:
        video_gid = grp.getgrnam('video').gr_gid
        if video_gid in os.getgroups():
            return True
        
        print("[RealSense] 警告: 当前用户不在 'video' 组中")
        print("建议运行: sudo usermod -a -G video $USER")
        print("然后重新登录系统")
        return False
    except KeyError:
        print("[RealSense] 'video' 组不存在,跳过权限检查")
        return True
    except Exception as e:
        print(f"[RealSense] 权限检查失败: {e}")
        return True


def reset_usb_devices() -> None:
    """
    尝试重置所有连接的 RealSense USB 设备
    """
    try:
        print("[RealSense] 正在尝试重置 USB 摄像头设备...")
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if not devices:
            print("[RealSense] 未找到 RealSense 设备")
            return
        
        for device in devices:
            try:
                device.hardware_reset()
                device_name = device.get_info(rs.camera_info.name)
                print(f"[RealSense] 已重置设备: {device_name}")
                time.sleep(2)  # 等待设备重新初始化
            except Exception as e:
                print(f"[RealSense] 重置设备失败: {e}")
                
    except Exception as e:
        print(f"[RealSense] 重置 USB 设备失败: {e}")


def get_first_device(context: rs.context) -> Optional[rs.device]:
    """
    返回第一个 RealSense 设备
    
    Args:
        context: RealSense 上下文对象
        
    Returns:
        Optional[rs.device]: 设备对象,如果没有设备则返回 None
    """
    devices = context.query_devices()
    if len(devices) == 0:
        return None
    return devices[0]


def colourise_depth(depth_frame: rs.depth_frame) -> np.ndarray:
    """
    将深度帧转换为伪彩色图像(用于可视化)
    
    Args:
        depth_frame: RealSense 深度帧
        
    Returns:
        np.ndarray: 伪彩色深度图像 (BGR 格式)
    """
    depth_data = np.asanyarray(depth_frame.get_data())
    
    # 方案1: 提高对比度 (推荐)
    # 使用更大的 alpha 值增强色彩层次
    # depth_image = cv2.convertScaleAbs(depth_data, alpha=0.08)  # 从 0.03 改为 0.08
    # return cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
    
    # 方案2: 自适应归一化 (色彩最丰富)
    valid_depth = depth_data[depth_data > 0]
    if len(valid_depth) > 0:
        min_depth = np.percentile(valid_depth, 5)   # 忽略最近 5%
        max_depth = np.percentile(valid_depth, 95)  # 忽略最远 5%
        depth_normalized = np.clip((depth_data - min_depth) / (max_depth - min_depth + 1e-6), 0, 1)
        depth_image = (depth_normalized * 255).astype(np.uint8)
    else:
        depth_image = np.zeros_like(depth_data, dtype=np.uint8)
    return cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)


# ============================================================================
# RealSense 摄像头类
# ============================================================================

class RealSenseCamera:
    """
    RealSense 摄像头封装类
    
    支持彩色流和深度流同时捕获,使用后台线程持续采集以提高帧率。
    
    Attributes:
        width (int): 彩色流宽度
        height (int): 彩色流高度
        fps (int): 帧率
        depth_scale (float): 深度尺度系数 (深度值 × 尺度 = 米)
        is_running (bool): 摄像头运行状态
    """
    
    # 支持的深度流分辨率 (按优先级排序)
    DEPTH_RESOLUTIONS = [
        (1280, 720),   # 高分辨率
        (848, 480),    # 中等分辨率
        (640, 480),    # 标准分辨率
        (424, 240),    # 低分辨率
    ]
    
    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30):
        """
        初始化 RealSense 摄像头
        
        Args:
            width: 彩色流宽度 (默认 1280)
            height: 彩色流高度 (默认 720)
            fps: 帧率 (默认 30)
        """
        self.width = width
        self.height = height
        self.fps = fps
        
        # 摄像头组件
        self.pipeline: Optional[rs.pipeline] = None
        self.config = rs.config()
        self.align: Optional[rs.align] = None
        self.colorizer = rs.colorizer()
        
        # 深度后处理滤波器
        self.spatial_filter: Optional[rs.spatial_filter] = None
        self.temporal_filter: Optional[rs.temporal_filter] = None
        
        # 深度尺度
        self.depth_scale: float = 1.0
        
        # 自动选择深度流分辨率
        self.depth_width, self.depth_height = self._get_optimal_depth_resolution()
        
        # 后台捕获线程
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        # 最新帧缓存
        self.latest_frames: Dict[str, Optional[np.ndarray]] = {
            "rgb": None,
            "depth_raw": None,
            "depth_colored": None,
        }
    
    def _get_optimal_depth_resolution(self) -> Tuple[int, int]:
        """
        根据彩色流分辨率自动选择最佳深度流分辨率
        
        Returns:
            Tuple[int, int]: 深度流的 (宽度, 高度)
        """
        color_pixels = self.width * self.height
        
        for depth_w, depth_h in self.DEPTH_RESOLUTIONS:
            depth_pixels = depth_w * depth_h
            # 允许深度分辨率略大于彩色分辨率 (20% 余量)
            if depth_pixels <= color_pixels * 1.2:
                return depth_w, depth_h
        
        # 默认使用最小分辨率
        return self.DEPTH_RESOLUTIONS[-1]
    
    def _print_device_info(self, device: rs.device) -> None:
        """打印设备信息"""
        print(f"[RealSense] 设备名称: {device.get_info(rs.camera_info.name)}")
        print(f"[RealSense] 序列号: {device.get_info(rs.camera_info.serial_number)}")
        print(f"[RealSense] 固件版本: {device.get_info(rs.camera_info.firmware_version)}")
    
    def _print_supported_resolutions(self, device: rs.device) -> None:
        """打印设备支持的分辨率"""
        print("[RealSense] 设备支持的分辨率:")
        for sensor in device.query_sensors():
            sensor_name = sensor.get_info(rs.camera_info.name)
            print(f"  传感器: {sensor_name}")
            
            for profile in sensor.get_stream_profiles():
                if profile.is_video_stream_profile():
                    v_profile = profile.as_video_stream_profile()
                    stream_type = v_profile.stream_type().name
                    width = v_profile.width()
                    height = v_profile.height()
                    fps = v_profile.fps()
                    fmt = v_profile.format().name
                    print(f"    - {stream_type}: {width}x{height} @ {fps}Hz ({fmt})")
    
    def start(self) -> bool:
        """
        启动摄像头并开始后台捕获
        
        Returns:
            bool: 启动成功返回 True
        """
        # 权限检查
        check_video_permissions()
        
        max_retries = 3
        for attempt in range(max_retries):
            try:
                print(f"[RealSense] 正在启动... (尝试 {attempt + 1}/{max_retries})")
                
                # 检查设备可用性
                if not check_device_availability():
                    print("[RealSense] 设备被占用,尝试重置...")
                    reset_usb_devices()
                
                # 检查设备连接
                ctx = rs.context()
                device = get_first_device(ctx)
                if device is None:
                    raise RuntimeError("未找到 RealSense 设备")
                
                # 打印设备信息
                self._print_device_info(device)
                
                # 配置彩色流
                self.config.enable_stream(
                    rs.stream.color, 
                    self.width, 
                    self.height, 
                    rs.format.bgr8, 
                    self.fps
                )
                print(f"[RealSense] 彩色流配置: {self.width}×{self.height} @ {self.fps}FPS")
                
                # 配置深度流
                self.config.enable_stream(
                    rs.stream.depth, 
                    self.depth_width, 
                    self.depth_height, 
                    rs.format.z16, 
                    self.fps
                )
                print(f"[RealSense] 深度流配置: {self.depth_width}×{self.depth_height} @ {self.fps}FPS")
                
                # 启动管道
                self.pipeline = rs.pipeline(ctx)
                profile = self.pipeline.start(self.config)
                
                # 初始化深度处理组件
                self.align = rs.align(rs.stream.color)
                self.spatial_filter = rs.spatial_filter()
                self.temporal_filter = rs.temporal_filter()
                
                # 获取深度尺度
                device = profile.get_device()
                depth_sensor = device.first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()
                print(f"[RealSense] 深度尺度: {self.depth_scale} (深度值 × 尺度 = 米)")
                
                # 获取相机内参
                color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile()
                depth_intr = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                color_intrinsics = color_intr.get_intrinsics()
                depth_intrinsics = depth_intr.get_intrinsics()
                
                # 保存深度相机内参供外部使用
                self.depth_intrinsics = depth_intrinsics
                
                print(f"[RealSense] 彩色相机内参: {color_intrinsics.width}×{color_intrinsics.height}, "
                      f"fx={color_intrinsics.fx:.1f}, fy={color_intrinsics.fy:.1f}")
                print(f"[RealSense] 深度相机内参: {depth_intrinsics.width}×{depth_intrinsics.height}, "
                      f"fx={depth_intrinsics.fx:.1f}, fy={depth_intrinsics.fy:.1f}")
                
                # 启动后台捕获线程
                self._running = True
                self._thread = threading.Thread(target=self._capture_loop, daemon=True)
                self._thread.start()
                
                print("[RealSense] 摄像头已启动,后台线程正在捕获帧")
                return True
                
            except RuntimeError as e:
                error_msg = str(e)
                print(f"[RealSense] 启动失败: {e}")
                
                if "Device or resource busy" in error_msg or "xioctl" in error_msg:
                    if attempt < max_retries - 1:
                        print("[RealSense] 检测到设备忙碌,正在重置并重试...")
                        if self.pipeline:
                            try:
                                self.pipeline.stop()
                            except:
                                pass
                        reset_usb_devices()
                        time.sleep(2)
                    else:
                        print("[RealSense] 所有重试均失败")
                        print("\n故障排除建议:")
                        print("1. 确保没有其他程序在使用摄像头")
                        print("2. 检查 USB 连接是否稳定")
                        print("3. 尝试重新插拔摄像头")
                        print("4. 重启系统以清理设备状态")
                        return False
                else:
                    if attempt < max_retries - 1:
                        time.sleep(2)
                    else:
                        return False
                        
            except Exception as e:
                print(f"[RealSense] 启动失败: {e}")
                if attempt < max_retries - 1:
                    time.sleep(2)
                else:
                    return False
        
        return False
    
    def _capture_loop(self) -> None:
        """
        后台捕获循环
        
        持续从摄像头捕获帧并更新缓存。
        """
        while self._running:
            try:
                # 等待帧
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                
                # 对齐深度帧到彩色帧
                aligned_frames = self.align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # 应用深度后处理滤波
                depth_frame = self.spatial_filter.process(depth_frame)
                depth_frame = self.temporal_filter.process(depth_frame)
                
                # 转换图像数据
                color_image = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_colored = colourise_depth(depth_frame)
                
                # 线程安全更新缓存
                with self._lock:
                    self.latest_frames["rgb"] = color_image
                    self.latest_frames["depth_raw"] = depth_raw
                    self.latest_frames["depth_colored"] = depth_colored
                    
            except Exception as e:
                print(f"[RealSense] 捕获循环错误: {e}", file=sys.stderr)
                time.sleep(0.1)
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        获取最新的图像帧 (线程安全)
        
        Returns:
            Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
                - 彩色图像 (BGR 格式, uint8)
                - 原始深度图像 (uint16, 深度传感器原始值)
                - 伪彩色深度图像 (BGR 格式, uint8, 用于可视化)
        """
        with self._lock:
            return (
                self.latest_frames["rgb"].copy() if self.latest_frames["rgb"] is not None else None,
                self.latest_frames["depth_raw"].copy() if self.latest_frames["depth_raw"] is not None else None,
                self.latest_frames["depth_colored"].copy() if self.latest_frames["depth_colored"] is not None else None,
            )
    
    def get_frame(self) -> Optional[np.ndarray]:
        """
        仅获取彩色图像 (兼容旧接口)
        
        Returns:
            Optional[np.ndarray]: 彩色图像,失败返回 None
        """
        color_image, _, _ = self.get_frames()
        return color_image
    
    def save_images(self, output_dir: str, prefix: str = "") -> None:
        """
        保存当前图像到指定目录
        
        Args:
            output_dir: 输出目录路径
            prefix: 文件名前缀 (可选)
        """
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        
        if prefix:
            sample_id = f"{prefix}_{timestamp}"
        else:
            sample_id = timestamp
        
        rgb_img, depth_raw, depth_colored = self.get_frames()
        
        if rgb_img is not None:
            path = os.path.join(output_dir, f"{sample_id}_rgb.png")
            cv2.imwrite(path, rgb_img)
            print(f"[RealSense] RGB 图像已保存: {path}")
        
        if depth_raw is not None:
            path = os.path.join(output_dir, f"{sample_id}_depth.png")
            cv2.imwrite(path, depth_raw)
            print(f"[RealSense] 深度图像已保存: {path}")
        
        if depth_colored is not None:
            path = os.path.join(output_dir, f"{sample_id}_depth_colored.png")
            cv2.imwrite(path, depth_colored)
            print(f"[RealSense] 伪彩色深度图已保存: {path}")
    
    def stop(self) -> None:
        """
        停止摄像头并释放资源
        """
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            
            if self.pipeline:
                try:
                    self.pipeline.stop()
                except:
                    pass
            
            print("[RealSense] 摄像头已停止")
        
        self.pipeline = None
        self.is_running = False


# ============================================================================
# 简单的查看器示例 (可选)
# ============================================================================

def run_viewer(camera: RealSenseCamera, output_dir: str) -> None:
    """
    运行交互式 OpenCV 查看器
    
    Args:
        camera: RealSenseCamera 实例
        output_dir: 图像保存目录
    """
    print("\n--- 启动查看器 ---")
    print("按 'S' 保存图像 | 按 'ESC' 或 'Q' 退出")
    
    last_time = time.perf_counter()
    
    while True:
        rgb, depth_raw, depth_colored = camera.get_frames()
        
        if rgb is None or depth_colored is None:
            time.sleep(0.01)
            continue
        
        # 合成显示图像
        combo = cv2.hconcat([rgb, depth_colored])
        
        # 计算并显示 FPS
        current_time = time.perf_counter()
        fps = 1.0 / (current_time - last_time + 1e-6)
        last_time = current_time
        
        cv2.putText(combo, f"{fps:.1f} FPS", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        cv2.imshow("RealSense Viewer", combo)
        
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC or Q
            break
        elif key == ord('s'):  # S
            camera.save_images(output_dir)
    
    cv2.destroyAllWindows()


def main() -> None:
    """主函数 - 示例用法"""
    import argparse
    
    parser = argparse.ArgumentParser(description="RealSense 摄像头客户端")
    parser.add_argument("--width", type=int, default=1280, help="图像宽度")
    parser.add_argument("--height", type=int, default=720, help="图像高度")
    parser.add_argument("--fps", type=int, default=30, help="帧率")
    parser.add_argument("--output-dir", type=str, default="data/images", 
                       help="图像保存目录")
    args = parser.parse_args()
    
    camera = RealSenseCamera(width=args.width, height=args.height, fps=args.fps)
    camera._print_supported_resolutions(device=get_first_device(rs.context()))
    try:
        if camera.start():
            run_viewer(camera, args.output_dir)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"程序异常: {e}", file=sys.stderr)
    finally:
        camera.stop()
        print("程序已清理并退出")


if __name__ == "__main__":
    main()