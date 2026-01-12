#!/usr/bin/env python3
"""
screen_to_ik.py
===============

整合屏幕目标定位与IK求解的一站式工具

🆕 更新:
- 支持 Torso Z 验证的深度获取
- 使用中值填补策略处理反光区域
"""

import numpy as np
import ikpy.chain
from typing import List, Optional, Tuple
from pathlib import Path
import xml.etree.ElementTree as ET
import os
import sys

# 添加路径配置
# current_dir = os.path.dirname(os.path.abspath(__file__))
# if current_dir not in sys.path:
#     sys.path.append(current_dir)
from pathlib import Path
# project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 🆕 导入升级版定位器
from screen_target_locator import ScreenTargetLocator
from touch_exceptions import (
    CameraError, 
    TargetNotFoundError, 
    DepthAcquisitionError, 
    IKSolutionError,
    SafetyLimitError
)
from touch_exceptions import (
    CameraError, 
    TargetNotFoundError, 
    DepthAcquisitionError, 
    IKSolutionError,
    SafetyLimitError
)


class ScreenToIKSolver:
    """屏幕目标到IK求解的集成器"""
    
    def __init__(self, 
                 urdf_file: Optional[str] = None,
                 yolo_server: str = "http://192.168.77.103:28000",
                 current_joint_state: Optional[List[float]] = None,
                 expected_torso_z: float = -0.17,       # 🆕 屏幕Z基准
                 torso_z_tolerance: float = 0.05,       # 🆕 Z容差
                 measurement_error: Optional[List[float]] = None): # 🆕 测量误差
        """
        初始化求解器
        
        Args:
            urdf_file: URDF模型文件路径 (默认使用同目录下的 g1.urdf)
            yolo_server: YOLO服务地址
            current_joint_state: 当前关节角度 [7维]
            expected_torso_z: 屏幕平面Torso Z基准值 (米)
            torso_z_tolerance: Z值容差 (米)
            measurement_error: 测量误差修正向量 [x, y, z]
        """
        # 处理 URDF 路径
        if urdf_file is None:
            urdf_file = str(Path(__file__).parent / "g1.urdf")
        elif not os.path.isabs(urdf_file):
            urdf_file = str(Path(__file__).parent / urdf_file)
            
        if not os.path.exists(urdf_file):
             print(f"⚠️ 警告: URDF文件未找到: {urdf_file}")

        # 🆕 初始化升级版目标定位器
        self.locator = ScreenTargetLocator(
            yolo_server_url=yolo_server,
            expected_torso_z=expected_torso_z,
            torso_z_tolerance=torso_z_tolerance
        )

        # 保存误差修正向量
        if measurement_error is None:
             # 默认为走跑模式误差 (根据 offset_data.md)
             self.measurement_error = np.array([0.01, -0.08, 0.25])
        else:
             self.measurement_error = np.array(measurement_error)
        
        print(f"🔧 IK求解器配置:")
        print(f"   - Torso Z基准: {expected_torso_z:.3f}m")
        print(f"   - 测量误差修正: {self.measurement_error.tolist()}")

        
        # 构建运动学链
        print("🔧 正在构建运动学链条...")
        self.chain = self._build_chain_from_urdf(urdf_file, "torso_link", "left_hand_palm_link")
        print(f"   ✅ 链条构建成功,共 {len(self.chain.links)} 个环节")
        
        # 设置当前状态
        if current_joint_state is None:
            current_joint_state = [
                0.002999999999999989,
                0.168000000000001,
                -0.03099999999999975,
                -0.13399999999999967,
                1.41,
                0.027,
                -0.008
            ]
        
        self.current_state = [0.0] + current_joint_state + [0.0]
        
        # 提取当前姿态约束
        current_frame = self.chain.forward_kinematics(self.current_state)
        self.constraint_orientation = current_frame[:3, :3]
        
        print(f"   ✅ 已锁定当前手掌姿态")
    
    def _build_chain_from_urdf(self, urdf_file, base_link, tip_link):
        """构建运动学链 (保持不变)"""
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        link_parent_joint = {}
        joints = {}
        
        for joint in root.findall('joint'):
            name = joint.get('name')
            child = joint.find('child').get('link')
            parent = joint.find('parent').get('link')
            joint_type = joint.get('type', 'fixed')
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]
            axis_elem = joint.find('axis')
            axis = [float(x) for x in axis_elem.get('xyz').split()] if axis_elem is not None else [0, 0, 0]
            limit = joint.find('limit')
            lower = float(limit.get('lower', -3.14)) if limit is not None else -np.inf
            upper = float(limit.get('upper', 3.14)) if limit is not None else np.inf
            
            joints[name] = {
                'type': joint_type, 'xyz': xyz, 'rpy': rpy,
                'axis': axis, 'bounds': (lower, upper),
                'parent_link': parent, 'child_link': child
            }
            link_parent_joint[child] = name
        
        chain_joints = []
        current_link = tip_link
        while current_link != base_link:
            if current_link not in link_parent_joint:
                raise ValueError(f"断链! 无法从 {tip_link} 回溯到 {base_link}")
            joint_name = link_parent_joint[current_link]
            joint_data = joints[joint_name]
            chain_joints.insert(0, (joint_name, joint_data))
            current_link = joint_data['parent_link']
        
        ikpy_links = [ikpy.link.OriginLink()]
        active_mask = [False]
        
        for name, data in chain_joints:
            is_fixed = (data['type'] == 'fixed')
            link = ikpy.link.URDFLink(
                name=name,
                origin_translation=data['xyz'],
                origin_orientation=data['rpy'],
                rotation=None if is_fixed else data['axis'],
                bounds=data['bounds'],
                joint_type='fixed' if is_fixed else 'revolute'
            )
            ikpy_links.append(link)
            active_mask.append(not is_fixed)
        
        return ikpy.chain.Chain(ikpy_links, name="g1_left_arm", active_links_mask=active_mask)
    
    def solve_for_target(self, target_index: int, apply_error_correction: bool = True) -> Tuple[List[float], np.ndarray]:
        """
        为指定屏幕区域求解IK
        
        Raises:
            CameraError, TargetNotFoundError, DepthAcquisitionError, IKSolutionError
        
        Returns:
            Tuple[List[float], np.ndarray]: (7维关节角度, Torso坐标)
        """
        print(f"\n{'='*60}")
        print(f"🎯 开始为目标区域 {target_index} 求解IK")
        print(f"{'='*60}")
        
        # 1. 启动摄像头
        if not self.locator.camera.start():
            print("❌ [IK] 摄像头启动失败")
            raise CameraError("摄像头启动失败")
        
        # 🆕 初始化DepthHelper (必须在相机启动后)
        self.locator._init_depth_helper()
        
        try:
            import time
            print("⏳ 等待摄像头稳定...")
            time.sleep(2)
            
            color_image, depth_raw, _ = self.locator.camera.get_frames()
            
            if color_image is None or depth_raw is None:
                print("❌ [IK] 无法获取图像 (Color或Depth为空)")
                raise CameraError("无法获取图像 (Color或Depth为空)")
            
            # 2. 🆕 使用升级版检测 (内置Torso Z验证)
            # 注意: detect_and_locate 现在会抛出异常而不是返回 None
            result = self.locator.detect_and_locate(color_image, depth_raw, target_index)
            
            target_pos_camera = np.array(result['torso_coord'])
            
            # 🆕 显示检测方法
            method = result.get('method', 'unknown')
            z_dev = result.get('torso_z_deviation', 0) * 100
            print(f"\n🔧 深度获取方法: {method}")
            print(f"📊 Torso Z偏差: {z_dev:.1f}cm")
            
            # 3. 应用误差修正 (可选)
            if apply_error_correction:
                # 使用初始化时配置的误差向量
                measurement_error = self.measurement_error
                target_pos = target_pos_camera + measurement_error
                print(f"📏 已应用误差修正: {measurement_error}")
            else:
                target_pos = target_pos_camera
            
            print(f"🎯 目标坐标 (Torso系): {target_pos}")
            
            # 4. 执行IK求解
            print(f"\n🔧 开始IK求解...")
            print(f"   - 目标位置: {target_pos}")
            print(f"   - 姿态约束: 保持当前手掌方向")
            
            ik_solution = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=self.constraint_orientation,
                orientation_mode="all",
                initial_position=self.current_state
            )
            
            # 5. 验证结果
            final_frame = self.chain.forward_kinematics(ik_solution)
            final_pos = final_frame[:3, 3]
            pos_error = np.linalg.norm(final_pos - target_pos)
            
            print(f"\n📊 求解验证:")
            print(f"   目标坐标: {target_pos}")
            print(f"   实际到达: {final_pos}")
            print(f"   位置误差: {pos_error*1000:.2f} mm")
            
            if pos_error > 0.05:
                print(f"❌ [IK] 位置误差过大: {pos_error:.3f}m > 0.05m")
                raise IKSolutionError(f"位置误差过大 ({pos_error:.3f}m > 0.05m), 可能超出工作空间")
            else:
                print(f"   ✅ 位置误差在可接受范围内")
            
            # 6. 提取7维关节角度
            joint_angles = [ik_solution[i] for i in range(1, len(ik_solution)-1)]
            
            # 保存结果
            self._save_ik_result(target_index, target_pos, joint_angles, result)
            
            return joint_angles, target_pos
            
        finally:
            self.locator.camera.stop()
    
    def _save_ik_result(self, target_index: int, target_pos: np.ndarray, 
                       joint_angles: List[float], detection_result: dict):
        """保存IK结果到文件"""
        output_dir = Path("data/ik_results")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        from datetime import datetime
        import json
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        result = {
            "timestamp": timestamp,
            "target_index": target_index,
            "target_position_torso": target_pos.tolist(),
            "joint_angles": joint_angles,
            "detection_info": {
                "pixel_coord": detection_result['pixel_coord'],
                "camera_coord": detection_result['camera_coord'],
                "depth_meters": detection_result['depth_meters'],
                "method": detection_result.get('method', 'unknown'),          # 🆕
                "torso_z_deviation": detection_result.get('torso_z_deviation', 0)  # 🆕
            }
        }
        
        json_path = output_dir / f"ik_target_{target_index}_{timestamp}.json"
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        
        print(f"\n💾 结果已保存: {json_path}")


def main():
    """命令行接口"""
    import argparse
    
    parser = argparse.ArgumentParser(description="屏幕目标IK求解器")
    parser.add_argument("target_index", type=int, help="目标区域编号 (0-35)")
    parser.add_argument("--urdf", type=str, default="g1.urdf", help="URDF文件路径")
    parser.add_argument("--server", type=str, default="http://192.168.77.103:28000",
                       help="YOLO服务地址")
    parser.add_argument("--no-correction", action="store_true",
                       help="禁用误差修正")
    parser.add_argument("--current-state", type=float, nargs=7,
                       help="当前关节状态 (7个浮点数)")
    # 🆕 新增参数
    parser.add_argument("--torso-z", type=float, default=-0.17,
                       help="屏幕Torso Z基准值 (米)")
    parser.add_argument("--z-tolerance", type=float, default=0.05,
                       help="Z值容差 (米)")
    
    args = parser.parse_args()
    
    # 🆕 传入Z基准参数
    solver = ScreenToIKSolver(
        urdf_file=args.urdf,
        yolo_server=args.server,
        current_joint_state=args.current_state,
        expected_torso_z=args.torso_z,
        torso_z_tolerance=args.z_tolerance
    )
    
    try:
        result = solver.solve_for_target(
            args.target_index,
            apply_error_correction=not args.no_correction
        )
        print("\n✅ 程序执行成功")
    except Exception as e:
        print(f"\n❌ 程序执行失败: {e}")


if __name__ == "__main__":
    main()