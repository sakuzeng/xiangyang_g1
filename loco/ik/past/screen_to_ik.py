#!/usr/bin/env python3
"""
screen_to_ik.py
===============

整合屏幕目标定位与IK求解的一站式工具

功能流程:
1. 输入屏幕区域编号 (0-35)
2. 调用YOLO检测获取目标中心像素坐标
3. 转换为Torso坐标系
4. 执行IK求解 (保持当前姿态)
5. 输出可直接使用的关节角度

依赖模块:
- screen_target_locator.ScreenTargetLocator (目标定位)
- ikpy (逆运动学求解)

from screen_to_ik import ScreenToIKSolver

# 初始化求解器
solver = ScreenToIKSolver()

# 为目标区域10求解
joint_angles = solver.solve_for_target(10)

# 直接使用结果
if joint_angles:
    print(f"求解成功: {joint_angles}")
"""

import numpy as np
import ikpy.chain
from typing import List, Optional, Tuple
from pathlib import Path
import xml.etree.ElementTree as ET

# 导入自定义模块
from screen_target_locator import ScreenTargetLocator


class ScreenToIKSolver:
    """屏幕目标到IK求解的集成器"""
    
    def __init__(self, 
                 urdf_file: str = "g1.urdf",
                 yolo_server: str = "http://192.168.77.103:28000",
                 current_joint_state: Optional[List[float]] = None):
        """
        初始化求解器
        
        Args:
            urdf_file: URDF模型文件路径
            yolo_server: YOLO服务地址
            current_joint_state: 当前关节角度 [7维] (如果为None则使用默认姿态)
        """
        # 1. 初始化目标定位器
        self.locator = ScreenTargetLocator(yolo_server)
        
        # 2. 构建运动学链
        print("🔧 正在构建运动学链条...")
        self.chain = self._build_chain_from_urdf(urdf_file, "torso_link", "left_hand_palm_link")
        print(f"   ✅ 链条构建成功,共 {len(self.chain.links)} 个环节")
        
        # 3. 设置当前状态
        if current_joint_state is None:
            # 默认姿态
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
        
        # 4. 提取当前姿态约束
        current_frame = self.chain.forward_kinematics(self.current_state)
        self.constraint_orientation = current_frame[:3, :3]
        
        print(f"   ✅ 已锁定当前手掌姿态")
    
    def _build_chain_from_urdf(self, urdf_file, base_link, tip_link):
        """构建运动学链 (复用g1_ik_orientation.py的逻辑)"""
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
        
        # 回溯链条
        chain_joints = []
        current_link = tip_link
        while current_link != base_link:
            if current_link not in link_parent_joint:
                raise ValueError(f"断链! 无法从 {tip_link} 回溯到 {base_link}")
            joint_name = link_parent_joint[current_link]
            joint_data = joints[joint_name]
            chain_joints.insert(0, (joint_name, joint_data))
            current_link = joint_data['parent_link']
        
        # 构建ikpy链
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
    
    def solve_for_target(self, target_index: int, apply_error_correction: bool = True) -> Optional[Tuple[List[float], np.ndarray]]:
        """
        为指定屏幕区域求解IK
        
        Args:
            target_index: 目标区域编号 (0-35)
            apply_error_correction: 是否应用测量误差修正
        
        Returns:
            Tuple[List[float], np.ndarray]: (7维关节角度, Torso坐标) 或 None
        """
        print(f"\n{'='*60}")
        print(f"🎯 开始为目标区域 {target_index} 求解IK")
        print(f"{'='*60}")
        
        # 1. 启动摄像头并定位目标
        if not self.locator.camera.start():
            print("❌ 摄像头启动失败")
            return None
        
        try:
            import time
            print("⏳ 等待摄像头稳定...")
            time.sleep(2)
            
            color_image, depth_raw, _ = self.locator.camera.get_frames()
            
            if color_image is None or depth_raw is None:
                print("❌ 无法获取图像")
                return None
            
            # 2. 检测目标并获取Torso坐标
            result = self.locator.detect_and_locate(color_image, depth_raw, target_index)
            
            if not result:
                print("❌ 目标定位失败")
                return None
            
            target_pos_camera = np.array(result['torso_coord'])
            
            # 3. 应用误差修正 (可选)
            if apply_error_correction:
                measurement_error = np.array([-0.01, -0.07, 0.23])
                target_pos = target_pos_camera + measurement_error
                print(f"\n📏 已应用误差修正: {measurement_error}")
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
                print(f"   ⚠️ 位置误差较大，可能超出机械臂工作空间")
            else:
                print(f"   ✅ 位置误差在可接受范围内")
            
            # 6. 提取7维关节角度
            joint_angles = [ik_solution[i] for i in range(1, len(ik_solution)-1)]
            
            # 保存结果
            self._save_ik_result(target_index, target_pos, joint_angles, result)
            
            # 🆕 同时返回关节角度和Torso坐标
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
                "depth_meters": detection_result['depth_meters']
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
    
    args = parser.parse_args()
    
    # 初始化求解器
    solver = ScreenToIKSolver(
        urdf_file=args.urdf,
        yolo_server=args.server,
        current_joint_state=args.current_state
    )
    
    # 执行求解
    result = solver.solve_for_target(
        args.target_index,
        apply_error_correction=not args.no_correction
    )
    
    if result:
        print("\n✅ 程序执行成功")
    else:
        print("\n❌ 程序执行失败")


if __name__ == "__main__":
    main()