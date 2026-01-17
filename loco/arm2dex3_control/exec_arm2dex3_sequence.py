#!/usr/bin/env python3
"""
手臂+灵巧手联合序列控制器 - 非交互式版本
功能：
- 同时控制手臂和灵巧手的姿态序列
- 支持混合序列编排
- 自动正向+反向归位
- 基于 _current_jpos_des 维护状态
"""
import sys
import time
import json
from pathlib import Path
from typing import List, Dict, Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
from pathlib import Path
# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from xiangyang.loco.common.robot_state_manager import robot_state


class ArmHandSequence:
    """手臂+灵巧手联合序列控制器"""
    
    def __init__(self, arm: str = "left", hand: str = "left", interface: str = "eth0"):
        self.arm = arm
        self.hand = hand
        self.interface = interface
        
        # 客户端
        self.arm_client = None
        self.hand_client = None
        
        # 姿态文件
        self.arm_pose_file = Path(f"../arm_control/saved_poses/{arm}_arm_poses.json")
        self.hand_pose_file = Path(f"../dex3_control/saved_poses/{hand}_hand_poses.json")
        
        # 姿态数据
        self.arm_poses = {}
        self.hand_poses = {}
        
    def initialize(self) -> bool:
        """初始化"""
        try:
            print(f"🔧 初始化 {self.arm.upper()} 手臂 + {self.hand.upper()} 手...")
            ChannelFactoryInitialize(0, self.interface)
            
            # 初始化手臂
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            if not self.arm_client.initialize_arms():
                print("❌ 手臂初始化失败")
                return False
            
            # 初始化灵巧手
            self.hand_client = robot_state.get_or_create_hand_client(
                hand=self.hand,
                interface=self.interface
            )
            if not self.hand_client.initialize_hand():
                print("❌ 灵巧手初始化失败")
                return False
            
            # 加载姿态文件
            if not self.arm_pose_file.exists():
                print(f"❌ 手臂姿态文件不存在: {self.arm_pose_file}")
                return False
            
            if not self.hand_pose_file.exists():
                print(f"❌ 灵巧手姿态文件不存在: {self.hand_pose_file}")
                return False
            
            with open(self.arm_pose_file, 'r') as f:
                self.arm_poses = json.load(f)
            
            with open(self.hand_pose_file, 'r') as f:
                self.hand_poses = json.load(f)
            
            print(f"✅ 初始化成功")
            print(f"   手臂姿态: {len(self.arm_poses)} 个")
            print(f"   灵巧手姿态: {len(self.hand_poses)} 个")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def execute_arm_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """执行手臂姿态"""
        if pose_name not in self.arm_poses:
            print(f"❌ 手臂姿态不存在: {pose_name}")
            return False
        
        positions = self.arm_poses[pose_name]['positions']
        offset = 0 if self.arm == 'left' else 7
        
        # 基于当前 _current_jpos_des 构建目标
        target_positions = self.arm_client._current_jpos_des.copy()
        target_positions[offset:offset+7] = positions
        
        print(f"  ▶️  [ARM] {pose_name}")
        
        try:
            self.arm_client.set_joint_positions(target_positions, speed_factor=speed_factor)
            print(f"  ✅ [ARM] 完成")
            return True
        except Exception as e:
            print(f"  ❌ [ARM] 失败: {e}")
            return False
    
    def execute_hand_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """执行灵巧手姿态"""
        if pose_name not in self.hand_poses:
            print(f"❌ 灵巧手姿态不存在: {pose_name}")
            return False
        
        positions = self.hand_poses[pose_name]['positions']
        
        print(f"  ▶️  [HAND] {pose_name}")
        
        try:
            self.hand_client.set_joint_positions(positions, speed_factor=speed_factor)
            print(f"  ✅ [HAND] 完成")
            return True
        except Exception as e:
            print(f"  ❌ [HAND] 失败: {e}")
            return False
    
    def execute_step(self, step: Dict[str, Any], speed_factor: float = 1.0) -> bool:
        """执行单个步骤"""
        step_type = step.get('type')
        
        if step_type == 'arm':
            return self.execute_arm_pose(step['pose'], speed_factor)
        elif step_type == 'hand':
            return self.execute_hand_pose(step['pose'], speed_factor)
        elif step_type == 'wait':
            duration = step.get('duration', 1.0)
            print(f"  ⏸️  等待 {duration}s...")
            time.sleep(duration)
            return True
        else:
            print(f"  ⚠️  未知步骤类型: {step_type}")
            return False
    
    def run_sequence(
        self, 
        sequence: List[Dict[str, Any]], 
        speed_factor: float = 1.0, 
        pause_time: float = 2.0
    ) -> bool:
        """执行联合序列（正向+反向）"""
        print("\n" + "="*70)
        print("🎬 开始执行联合序列")
        print("="*70)
        print(f"📝 总步骤: {len(sequence)}")
        print(f"⏱️  速度: {speed_factor}, 停留: {pause_time}s")
        print("="*70)
        
        try:
            # 使用双锁（手臂+灵巧手）
            with robot_state.safe_arm_control(arm=self.arm, source="joint_sequence", timeout=120.0):
                with robot_state.safe_hand_control(hand=self.hand, source="joint_sequence", timeout=120.0):
                    
                    # ========== 正向执行 ==========
                    print(f"\n🔵 正向执行 ({len(sequence)} 步)")
                    print("-"*70)
                    
                    for i, step in enumerate(sequence, 1):
                        print(f"[{i}/{len(sequence)}]", end=" ")
                        if not self.execute_step(step, speed_factor):
                            print("❌ 序列执行中断")
                            return False
                        
                        if i < len(sequence):
                            time.sleep(0.3)  # 步骤间延时
                    
                    print("-"*70)
                    print("✅ 正向执行完成")
                    
                    # 停留
                    if pause_time > 0:
                        print(f"\n⏸️  停留 {pause_time} 秒...")
                        time.sleep(pause_time)
                    
                    # ========== 反向执行 ==========
                    # 过滤掉 'wait' 类型的步骤
                    motion_steps = [s for s in sequence if s['type'] != 'wait']
                    reverse_sequence = list(reversed(motion_steps[:-1]))
                    
                    if reverse_sequence:
                        print(f"\n🔴 反向执行 ({len(reverse_sequence)} 步)")
                        print("-"*70)
                        
                        for i, step in enumerate(reverse_sequence, 1):
                            print(f"[{i}/{len(reverse_sequence)}]", end=" ")
                            if not self.execute_step(step, speed_factor):
                                print("❌ 反向执行中断")
                                return False
                            
                            if i < len(reverse_sequence):
                                time.sleep(0.3)
                        
                        print("-"*70)
                        print("✅ 反向执行完成")
                    
                    print("\n🏁 联合序列执行完毕")
                    return True
        
        except KeyboardInterrupt:
            print("\n\n⚠️  用户中断")
            return False
        except Exception as e:
            print(f"\n❌ 执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def shutdown(self):
        """关闭"""
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state(self.arm)
        
        if self.hand_client:
            self.hand_client.stop_control()
            robot_state.reset_hand_state(self.hand)
        
        print("✅ 已关闭")


def main():
    """主程序"""
    # ========== 🆕 在此配置联合序列 ==========
    ARM = "left"                     # 手臂: "left" 或 "right"
    HAND = "left"                    # 灵巧手: "left" 或 "right"
    INTERFACE = "eth0"               # 网络接口
    
    # 联合序列配置
    # type: 'arm' | 'hand' | 'wait'
    # pose: 姿态名称（对应保存文件中的键）
    # duration: 等待时间（仅 type='wait' 时有效）
    SEQUENCE = [
        {'type': 'arm', 'pose': 'phone_pre_1'},
        {'type': 'arm', 'pose': 'phone_pre_2'},
        {'type': 'arm', 'pose': 'phone_pre_3'},
        {'type': 'arm', 'pose': 'phone_pre_4'},
        {'type': 'arm', 'pose': 'phone_pre_5'},
        {'type': 'arm', 'pose': 'phone_pre_6'},
        {'type': 'arm', 'pose': 'phone_pre_7'},
        # {'type': 'wait', 'duration': 0.5},  # 等待0.5秒
        {'type': 'hand', 'pose': 'phone_pre_1'},  # 灵巧手姿态
        # {'type': 'wait', 'duration': 0.5},
        {'type': 'arm', 'pose': 'phone_pre_8'},
    ]
    
    SPEED_FACTOR = 1.0               # 运动速度 (0.1-1.0)
    PAUSE_TIME = 2.0                 # 最后姿态停留时间（秒）
    # ==========================================
    
    print("="*70)
    print("🎬 手臂+灵巧手联合序列控制器")
    print("="*70)
    print(f"💪 手臂: {ARM.upper()}")
    print(f"🖐️  手: {HAND.upper()}")
    print(f"🌐 接口: {INTERFACE}")
    print("="*70)
    
    controller = ArmHandSequence(arm=ARM, hand=HAND, interface=INTERFACE)
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        # 执行序列
        success = controller.run_sequence(
            sequence=SEQUENCE,
            speed_factor=SPEED_FACTOR,
            pause_time=PAUSE_TIME
        )
        
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