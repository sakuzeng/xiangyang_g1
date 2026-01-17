#!/usr/bin/env python3
"""
双臂姿态序列控制器
功能：
- 同时控制左右手臂
- 从 saved_poses 加载预定义姿态
- 支持 "keep" (保持), "nature" (自然位) 和自定义姿态名称
"""
import sys
import time
import json
import os
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Union

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xiangyang.loco.common.robot_state_manager import robot_state
from unitree_sdk2py.arm.arm_client import G1ArmGestures

class DualArmPoseSequence:
    def __init__(self, interface: str = "eth0"):
        self.interface = interface
        self.arm_client = None
        self.left_poses = {}
        self.right_poses = {}
        self.pose_dir = Path(os.path.dirname(os.path.abspath(__file__))) / "saved_poses"
        
    def initialize(self) -> bool:
        """初始化机器人和加载姿态"""
        try:
            print("🔧 初始化双臂控制器...")
            ChannelFactoryInitialize(0, self.interface)
            
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            
            if not self.arm_client.initialize_arms():
                print("❌ 初始化失败")
                return False
            
            # 加载姿态文件
            self._load_poses()
            
            print("✅ 初始化成功")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _load_poses(self):
        """加载左右臂姿态文件"""
        left_file = self.pose_dir / "left_arm_poses.json"
        right_file = self.pose_dir / "right_arm_poses.json"
        
        if left_file.exists():
            with open(left_file, 'r', encoding='utf-8') as f:
                self.left_poses = json.load(f)
            print(f"📥 已加载左臂姿态: {len(self.left_poses)} 个")
        
        if right_file.exists():
            with open(right_file, 'r', encoding='utf-8') as f:
                self.right_poses = json.load(f)
            print(f"📥 已加载右臂姿态: {len(self.right_poses)} 个")

    def _get_joint_positions(self, arm: str, pose_name: Optional[str], current_full_positions: List[float]) -> List[float]:
        """
        获取单臂的关节目标位置
        arm: 'left' or 'right'
        pose_name: 姿态名称, 'nature', 'keep', None
        current_full_positions: 当前所有关节位置 (14维)
        """
        offset = 0 if arm == 'left' else 7
        current_arm_positions = current_full_positions[offset:offset+7]
        
        # 1. 保持当前位置
        if pose_name is None or pose_name.lower() == "keep":
            return current_arm_positions
            
        # 2. 自然位 (从 G1ArmGestures 获取)
        if pose_name.lower() == "nature":
            nature_full = G1ArmGestures.get_pose("nature")
            return nature_full[offset:offset+7]
            
        # 3. 从文件加载的姿态
        poses_dict = self.left_poses if arm == 'left' else self.right_poses
        if pose_name in poses_dict:
            return poses_dict[pose_name]['positions']
            
        print(f"⚠️  警告: 未找到{arm}臂姿态 '{pose_name}'，保持当前位置")
        return current_arm_positions

    def execute_dual_pose(self, left_pose: str, right_pose: str, speed_factor: float = 1.0) -> bool:
        """执行双臂姿态"""
        # 获取当前期望位置作为基准
        current_positions = self.arm_client._current_jpos_des.copy()
        
        # 解析左右臂目标
        left_target = self._get_joint_positions('left', left_pose, current_positions)
        right_target = self._get_joint_positions('right', right_pose, current_positions)
        
        # 组合目标 (14维)
        target_positions = left_target + right_target
        
        # 确保长度正确
        if len(target_positions) != 14:
            print(f"❌ 错误: 目标关节数量不正确 ({len(target_positions)})")
            return False
            
        print(f"  ▶️  移动: 左[{left_pose or 'keep'}] + 右[{right_pose or 'keep'}]")
        
        try:
            self.arm_client.set_joint_positions(target_positions, speed_factor=speed_factor)
            return True
        except Exception as e:
            print(f"  ❌ 执行失败: {e}")
            return False

    def run_sequence(self, sequence: List[Tuple[str, str]], speed_factor: float = 1.0, pause_time: float = 1.0):
        """执行动作序列"""
        print("\n" + "="*70)
        print(f"🎬 开始执行双臂序列 ({len(sequence)} 步)")
        print("="*70)
        
        try:
            # 使用 safe_dual_arm_control 获取双臂权限
            with robot_state.safe_dual_arm_control(source="dual_sequence", timeout=120.0):
                for i, (left_pose, right_pose) in enumerate(sequence, 1):
                    print(f"[{i}/{len(sequence)}]", end=" ")
                    
                    if not self.execute_dual_pose(left_pose, right_pose, speed_factor):
                        print("❌ 序列中断")
                        return False
                    
                    if i < len(sequence):
                        time.sleep(pause_time)
                
                print("="*70)
                print("✅ 序列执行完成")
                return True
                
        except KeyboardInterrupt:
            print("\n⚠️  用户中断")
            return False
        except Exception as e:
            print(f"\n❌ 运行时错误: {e}")
            return False

    def shutdown(self):
        if self.arm_client:
            print("\n🔧 恢复自然位并停止...")
            self.arm_client.stop_control()

def main():
    # ========== 📝 配置区域 ==========
    INTERFACE = "eth0"
    SPEED_FACTOR = 1.0
    PAUSE_TIME = 1.0
    
    # 定义动作序列 [(左臂姿态, 右臂姿态)]
    # 可用值: 
    #   - 姿态名称 (如 "phone_pre_1", "hello_1")
    #   - "nature" (自然下垂)
    #   - "keep" 或 None (保持不动)
    SEQUENCE = [
        ("nature", "nature"),           # 1. 双臂自然下垂
        ("inte_up", "keep"),        # 2. 左臂准备1，右臂不动
        ("nature", "inte_up"),      # 3. 左臂准备2，右臂举手
        ("nature", "nature")            # 6. 回到自然位
    ]
    # ================================
    
    controller = DualArmPoseSequence(interface=INTERFACE)
    
    if not controller.initialize():
        sys.exit(1)
        
    try:
        controller.run_sequence(SEQUENCE, speed_factor=SPEED_FACTOR, pause_time=PAUSE_TIME)
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()