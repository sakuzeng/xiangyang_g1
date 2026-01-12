#!/usr/bin/env python3
"""
phone_touch_task.py
===================

手机触摸任务控制器

流程:
1. 初始化左臂和左灵巧手
2. 获取目标区域IK解
3. **显示Torso坐标并等待用户确认**
4. 执行触摸序列:
   - phone_prepare_1/2/final (手臂预备姿态)
   - phone_pre_1 (灵巧手姿态)
   - target_pos (移动到目标)
   - 左手腕yaw摆动 (-0.5rad → +0.5rad)
   - 左肘收缩 (-0.5rad)
   - 灵巧手恢复 (close)
   - 反向归位 (phone_prepare_final/2/1)
5. 🆕 紧急退出保护: 基于FK计算末端(x,z)坐标
   - 若 x>0 且 z>-0.1: 手臂在桌面上 → 肘关节收缩后归位
   - 其他情况: 直接shutdown
"""

import sys
import time
import json
from pathlib import Path
from typing import Optional, List, Tuple
import numpy as np

# SDK导入
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from common.robot_state_manager import robot_state

# 自定义模块
from screen_to_ik import ScreenToIKSolver


class PhoneTouchController:
    """手机触摸任务控制器"""
    
    def __init__(self, interface: str = "eth0"):
        self.interface = interface
        self.arm_client = None
        self.hand_client = None
        self.ik_solver = None
        
        # 姿态文件路径
        self.arm_pose_file = Path("../arm_control/saved_poses/left_arm_poses.json")
        self.hand_pose_file = Path("../dex3_control/saved_poses/left_hand_poses.json")
        self.arm_poses = {}
        self.hand_poses = {}
        
        # 任务状态
        self.emergency_exit = False
        self.target_joint_angles = None
        self.target_torso_coord = None
        
        # 🆕 安全阈值配置 (可根据实际情况调整)
        self.SAFE_X_THRESHOLD = 0.07   # X坐标阈值(米)
        self.SAFE_Z_THRESHOLD = -0.1  # Z坐标阈值(米)
    
    def initialize(self) -> bool:
        """初始化所有组件"""
        print("\n" + "="*70)
        print("🔧 初始化手机触摸控制器")
        print("="*70)
        
        try:
            # 1. 初始化通道
            ChannelFactoryInitialize(0, self.interface)
            
            # 2. 初始化左臂
            print("💪 初始化左臂...")
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            if not self.arm_client.initialize_arms():
                print("❌ 左臂初始化失败")
                return False
            
            # 3. 初始化左灵巧手
            print("✋ 初始化左灵巧手...")
            self.hand_client = robot_state.get_or_create_hand_client(
                hand="left",
                interface=self.interface
            )
            if not self.hand_client.initialize_hand():
                print("❌ 左灵巧手初始化失败")
                return False
            
            # 4. 加载姿态文件
            print("📂 加载姿态库...")
            if not self._load_poses():
                return False
            
            # 5. 初始化IK求解器
            print("🔧 初始化IK求解器...")
            self.ik_solver = ScreenToIKSolver()
            
            print("✅ 所有组件初始化成功\n")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _load_poses(self) -> bool:
        """加载姿态库"""
        try:
            # 手臂姿态
            if not self.arm_pose_file.exists():
                print(f"❌ 手臂姿态文件不存在: {self.arm_pose_file}")
                return False
            with open(self.arm_pose_file, 'r') as f:
                self.arm_poses = json.load(f)
            print(f"   ✅ 手臂姿态: {len(self.arm_poses)} 个")
            
            # 灵巧手姿态
            if not self.hand_pose_file.exists():
                print(f"❌ 灵巧手姿态文件不存在: {self.hand_pose_file}")
                return False
            with open(self.hand_pose_file, 'r') as f:
                self.hand_poses = json.load(f)
            print(f"   ✅ 灵巧手姿态: {len(self.hand_poses)} 个")
            
            return True
            
        except Exception as e:
            print(f"❌ 加载姿态失败: {e}")
            return False
    
    def _get_current_end_position(self) -> Optional[Tuple[float, float, float]]:
        """
        🆕 通过FK计算当前末端位置(x, y, z)
        
        Returns:
            Tuple[x, y, z]: Torso坐标系下的末端位置(米), 失败返回None
        """
        try:
            # 获取当前关节角度 (索引0-6是左臂)
            current_joints = self.arm_client._current_jpos_des[0:7]
            
            # 构造完整状态向量 [0.0, j1, j2, ..., j7, 0.0]
            full_state = [0.0] + list(current_joints) + [0.0]
            
            # FK计算 (使用ik_solver的运动学链)
            current_frame = self.ik_solver.chain.forward_kinematics(full_state)
            
            # 提取位置 (4x4变换矩阵的最后一列前三个元素)
            x = current_frame[0, 3]
            y = current_frame[1, 3]
            z = current_frame[2, 3]
            
            return (x, y, z)
            
        except Exception as e:
            print(f"⚠️  FK计算失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _check_need_lift(self) -> bool:
        """
        🆕 检查是否需要抬起手臂
        
        判断逻辑:
            - 若 x > SAFE_X_THRESHOLD 且 z > SAFE_Z_THRESHOLD: 手臂在桌面上 → 需要抬起
            - 其他情况: 直接关闭即可
        
        Returns:
            bool: True=需要抬起, False=直接关闭
        """
        pos = self._get_current_end_position()
        
        if pos is None:
            print("⚠️  无法获取末端位置,假定不需要抬起")
            return False
        
        x, y, z = pos
        
        print(f"\n📍 当前末端位置 (Torso坐标系):")
        print(f"   X = {x:+.4f} m")
        print(f"   Y = {y:+.4f} m")
        print(f"   Z = {z:+.4f} m")
        
        # 判断是否在桌面上
        need_lift = (x > self.SAFE_X_THRESHOLD) and (z > self.SAFE_Z_THRESHOLD)
        
        if need_lift:
            print(f"🚨 手臂在桌面上! (x={x:.3f} > {self.SAFE_X_THRESHOLD}, z={z:.3f} > {self.SAFE_Z_THRESHOLD})")
        else:
            print(f"✅ 手臂不在桌面上,可以直接关闭")
        
        return need_lift
    
    def _confirm_execution(self, target_index: int) -> bool:
        """
        显示目标信息并等待用户确认
        
        Returns:
            bool: True=继续执行, False=取消任务
        """
        print("\n" + "="*70)
        print("📋 任务确认信息")
        print("="*70)
        print(f"🎯 目标区域编号: {target_index}")
        print(f"\n📍 Torso坐标系目标位置:")
        print(f"   X = {self.target_torso_coord[0]:+.4f} m")
        print(f"   Y = {self.target_torso_coord[1]:+.4f} m")
        print(f"   Z = {self.target_torso_coord[2]:+.4f} m")
        
        print(f"\n🔧 关节角度 (弧度):")
        joint_names = [
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"
        ]
        for i, (name, angle) in enumerate(zip(joint_names, self.target_joint_angles)):
            print(f"   [{i}] {name:<20}: {angle:+.4f}")
        
        print("\n" + "="*70)
        print("📋 IK解算结果 (复制用)")
        print("="*70)
        
        new_joints = self.target_joint_angles
        
        # 紧凑格式
        print("\n# 紧凑格式(单行):")
        compact_str = "[" + ", ".join([f"{val:.6f}" for val in new_joints]) + "]"
        print(f"new_joints = {compact_str}")
        
        print("\n" + "="*70)
        print("⚠️  请确认以上信息是否正确!")
        print("="*70)
        
        while True:
            response = input("\n是否继续执行? (y/n): ").strip().lower()
            
            if response == 'y' or response == 'yes':
                print("✅ 用户确认,开始执行任务...")
                return True
            elif response == 'n' or response == 'no':
                print("❌ 用户取消,任务终止")
                return False
            else:
                print("⚠️  输入无效,请输入 y 或 n")
    
    def move_arm_to_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """移动手臂到指定姿态"""
        if pose_name not in self.arm_poses:
            print(f"❌ 手臂姿态不存在: {pose_name}")
            return False
        
        positions = self.arm_poses[pose_name]['positions']
        target = self.arm_client._current_jpos_des.copy()
        target[0:7] = positions
        
        print(f"  ▶️  移动手臂到: {pose_name}")
        try:
            self.arm_client.set_joint_positions(target, speed_factor=speed_factor)
            time.sleep(0.3)
            print(f"  ✅ 完成")
            return True
        except Exception as e:
            print(f"  ❌ 失败: {e}")
            return False
    
    def move_arm_to_angles(self, joint_angles: List[float], speed_factor: float = 1.0) -> bool:
        """移动手臂到指定关节角度"""
        target = self.arm_client._current_jpos_des.copy()
        target[0:7] = joint_angles
        
        print(f"  ▶️  移动到目标位置")
        try:
            self.arm_client.set_joint_positions(target, speed_factor=speed_factor)
            time.sleep(0.3)
            print(f"  ✅ 完成")
            return True
        except Exception as e:
            print(f"  ❌ 失败: {e}")
            return False
    
    def move_hand_to_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """移动灵巧手到指定姿态"""
        if pose_name not in self.hand_poses:
            print(f"❌ 灵巧手姿态不存在: {pose_name}")
            return False
        
        positions = self.hand_poses[pose_name]['positions']
        
        print(f"  ✋ 移动灵巧手到: {pose_name}")
        try:
            self.hand_client.set_joint_positions(
                positions=positions,
                duration=None,
                speed_factor=speed_factor
            )
            time.sleep(0.5)
            print(f"  ✅ 完成")
            return True
        except Exception as e:
            print(f"  ❌ 失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def adjust_single_joint(self, joint_index: int, delta_rad: float, speed_factor: float = 1.0):
        """调整单个关节角度"""
        target = self.arm_client._current_jpos_des.copy()
        target[joint_index] += delta_rad
        
        joint_names = [
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"
        ]
        
        print(f"  🔧 调整 {joint_names[joint_index]}: {delta_rad:+.2f} rad")
        try:
            self.arm_client.set_joint_positions(target, speed_factor=speed_factor)
            time.sleep(0.3)
            print(f"  ✅ 完成")
        except Exception as e:
            print(f"  ❌ 失败: {e}")
    
    def execute_task(self, target_index: int) -> bool:
        """执行完整任务流程"""
        print("\n" + "="*70)
        print(f"🎯 开始执行手机触摸任务 - 目标区域 {target_index}")
        print("="*70)
        
        try:
            # ========== 步骤0: 获取IK解 ==========
            print(f"\n【步骤0】获取目标区域 {target_index} 的IK解")
            print("-"*70)
            
            ik_result = self.ik_solver.solve_for_target(target_index)
            if not ik_result:
                print("❌ IK求解失败")
                return False
            
            self.target_joint_angles, self.target_torso_coord = ik_result
            
            # 用户确认
            if not self._confirm_execution(target_index):
                print("\n❌ 任务已取消")
                return False
            
            # ========== 正式开始执行 ==========
            with robot_state.safe_arm_control(arm="left", source="phone_touch", timeout=180.0):
                
                # ========== 步骤1: 预备姿态序列 ==========
                print(f"\n【步骤1】执行预备姿态序列")
                print("-"*70)
                
                prepare_sequence = ["phone_pre_1", "phone_pre_2", "phone_pre_3", "phone_pre_final"]
                for pose in prepare_sequence:
                    if not self.move_arm_to_pose(pose):
                        return False
                
                # ========== 步骤2: 灵巧手预备 ==========
                print(f"\n【步骤2】设置灵巧手姿态")
                print("-"*70)
                
                if not self.move_hand_to_pose("phone_pre_1"):
                    return False
                
                # ========== 步骤3: 移动到目标位置 ==========
                print(f"\n【步骤3】移动到目标位置")
                print("-"*70)
                
                if not self.move_arm_to_angles(self.target_joint_angles, speed_factor=1.0):
                    return False
                
                time.sleep(1.0)
                
                # ========== 步骤4: 手腕yaw摆动 ==========
                print(f"\n【步骤4】手腕yaw摆动测试")
                print("-"*70)
                
                WRIST_YAW_INDEX = 6
                
                print("  🔄 摆动 -0.55 rad")
                self.adjust_single_joint(WRIST_YAW_INDEX, -0.55)
                
                print("  🔄 摆动 +0.55 rad (归位)")
                self.adjust_single_joint(WRIST_YAW_INDEX, +0.55)
                
                # ========== 步骤5: 灵巧手恢复原位 ==========
                print(f"\n【步骤5】设置灵巧手恢复原位")
                print("-"*70)
                
                if not self.move_hand_to_pose("close"):
                    return False

                # ========== 步骤6: 肘关节收缩 ==========
                print(f"\n【步骤6】肘关节收缩")
                print("-"*70)
                
                ELBOW_INDEX = 3
                
                print("  💪 收缩 -0.5 rad")
                self.adjust_single_joint(ELBOW_INDEX, -0.5)
                
                # ========== 步骤7: 反向归位 ==========
                print(f"\n【步骤7】反向归位")
                print("-"*70)
                
                return_sequence = ["phone_pre_final", "phone_pre_3", "phone_pre_2", "phone_pre_1"]
                for pose in return_sequence:
                    if not self.move_arm_to_pose(pose):
                        return False
                
                print("\n🏁 任务执行完成!")
                return True
                
        except KeyboardInterrupt:
            print("\n\n⚠️  检测到键盘中断，执行安全退出...")
            self.emergency_exit = True
            self._safe_emergency_exit()
            return False
        except Exception as e:
            print(f"\n❌ 任务执行失败: {e}")
            import traceback
            traceback.print_exc()
            self.emergency_exit = True
            self._safe_emergency_exit()
            return False
    
    def _safe_emergency_exit(self):
        """
        🆕 智能紧急退出
        
        逻辑:
            1. 通过FK计算当前末端位置(x, z)
            2. 判断是否需要抬起:
               - 若 x > 0.07 且 z > -0.1: 肘关节收缩后逐步归位
               - 其他情况: 直接调用shutdown
        """
        print("\n" + "="*70)
        print("🚨 执行智能紧急退出")
        print("="*70)
        
        try:
            # 检查是否需要抬起
            if self._check_need_lift():
                print("\n🚑 需要先抬起手臂!")
                print("-"*70)
                
                # 1. 肘关节收缩
                print("💪 收缩肘关节...")
                self.adjust_single_joint(3, -0.5, speed_factor=1.0)
                time.sleep(0.5)
                
                # 2. 逐步归位
                print("🔄 逐步归位...")
                return_sequence = ["phone_pre_final", "phone_pre_3", "phone_pre_2", "phone_pre_1"]
                for pose in return_sequence:
                    self.move_arm_to_pose(pose, speed_factor=1.0)
                    time.sleep(0.5)
                
                print("✅ 安全退出完成")
            else:
                print("\n✅ 手臂安全,直接关闭")
                # 直接关闭(不执行额外动作)
            
        except Exception as e:
            print(f"⚠️  紧急退出过程出错: {e}")
            import traceback
            traceback.print_exc()
    
    def shutdown(self):
        """关闭所有控制器"""
        print("\n🔧 关闭控制器...")
        
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state("left")
        
        if self.hand_client:
            self.hand_client.stop_control()
            robot_state.reset_hand_state("left")
        
        print("✅ 已关闭")


def main():
    """主程序"""
    # ========== 配置参数 ==========
    TARGET_INDEX = 31      # 目标区域编号 (0-35)
    INTERFACE = "eth0"     # 网络接口
    # ==============================
    
    print("="*70)
    print("📱 手机触摸任务控制器 (智能紧急退出)")
    print("="*70)
    print(f"🎯 目标区域: {TARGET_INDEX}")
    print(f"🌐 网络接口: {INTERFACE}")
    print(f"⚠️  安全阈值: X > 0.07m 且 Z > -0.1m")
    print("="*70)
    
    controller = PhoneTouchController(interface=INTERFACE)
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        success = controller.execute_task(TARGET_INDEX)
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  收到中断信号")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()