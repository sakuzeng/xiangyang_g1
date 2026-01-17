#!/usr/bin/env python3
"""
灵巧手姿态序列控制器 - 非交互式版本
功能：
- 直接在代码中指定姿态序列
- 自动执行正向+反向归位
- 基于 _current_jpos_des 维护状态
"""
import sys
import time
import json
from pathlib import Path
from typing import List

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
from pathlib import Path

# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from xiangyang.loco.common.robot_state_manager import robot_state


class SimpleHandSequence:
    """简单灵巧手姿态序列控制器"""
    
    def __init__(self, hand: str = "left", interface: str = "eth0"):
        self.hand = hand
        self.interface = interface
        self.dex3 = None
        self.pose_file = Path(f"./saved_poses/{hand}_hand_poses.json")
        self.poses = {}
        
    def initialize(self) -> bool:
        """初始化"""
        try:
            print(f"🔧 初始化 {self.hand.upper()} 手...")
            ChannelFactoryInitialize(0, self.interface)
            
            self.dex3 = robot_state.get_or_create_hand_client(
                hand=self.hand,
                interface=self.interface
            )
            
            if not self.dex3.initialize_hand():
                print("❌ 初始化失败")
                return False
            
            # 加载姿态文件
            if not self.pose_file.exists():
                print(f"❌ 姿态文件不存在: {self.pose_file}")
                return False
            
            with open(self.pose_file, 'r') as f:
                self.poses = json.load(f)
            
            print(f"✅ 初始化成功，已加载 {len(self.poses)} 个姿态")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def execute_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """执行单个姿态"""
        if pose_name not in self.poses:
            print(f"❌ 姿态不存在: {pose_name}")
            return False
        
        positions = self.poses[pose_name]['positions']
        
        print(f"  ▶️  移动到: {pose_name}")
        
        try:
            # 直接发送命令（底层会更新 _current_jpos_des）
            self.dex3.set_joint_positions(positions, speed_factor=speed_factor)
            print(f"  ✅ 完成: {pose_name}")
            return True
        except Exception as e:
            print(f"  ❌ 失败: {e}")
            return False
    
    def run_sequence(self, sequence: List[str], speed_factor: float = 1.0, 
                     pause_time: float = 2.0):
        """执行姿态序列（正向+反向）"""
        print("\n" + "="*70)
        print("🎬 开始执行姿态序列")
        print("="*70)
        print(f"📝 序列: {' → '.join(sequence)}")
        print(f"⏱️  速度: {speed_factor}, 停留: {pause_time}s")
        print("="*70)
        
        try:
            with robot_state.safe_hand_control(hand=self.hand, source="pose_sequence", timeout=120.0):
                
                # ========== 正向执行 ==========
                print(f"\n🔵 正向执行 ({len(sequence)} 个姿态)")
                print("-"*70)
                
                for i, pose_name in enumerate(sequence, 1):
                    print(f"[{i}/{len(sequence)}]", end=" ")
                    if not self.execute_pose(pose_name, speed_factor):
                        print("❌ 序列执行中断")
                        return False
                    
                    if i < len(sequence):
                        time.sleep(0.5)
                
                print("-"*70)
                print("✅ 正向执行完成")
                
                # 停留
                if pause_time > 0:
                    print(f"\n⏸️  停留 {pause_time} 秒...")
                    time.sleep(pause_time)
                
                # ========== 反向执行 ==========
                reverse_sequence = list(reversed(sequence[:-1]))
                
                if reverse_sequence:
                    print(f"\n🔴 反向执行 ({len(reverse_sequence)} 个姿态)")
                    print("-"*70)
                    
                    for i, pose_name in enumerate(reverse_sequence, 1):
                        print(f"[{i}/{len(reverse_sequence)}]", end=" ")
                        if not self.execute_pose(pose_name, speed_factor):
                            print("❌ 反向执行中断")
                            return False
                        
                        if i < len(reverse_sequence):
                            time.sleep(0.5)
                    
                    print("-"*70)
                    print("✅ 反向执行完成")
                
                print("\n🏁 姿态序列执行完毕")
                return True
                
        except KeyboardInterrupt:
            print("\n\n⚠️  用户中断")
            return False
        except Exception as e:
            print(f"\n❌ 执行失败: {e}")
            return False
    
    def shutdown(self):
        """关闭"""
        if self.dex3:
            print("\n🔧 关闭控制器...")
            self.dex3.stop_control()
            robot_state.reset_hand_state(self.hand)
            print("✅ 已关闭")


def main():
    """主程序"""
    # ========== 🆕 在此配置姿态序列 ==========
    HAND = "left"                    # 灵巧手选择: "left" 或 "right"
    INTERFACE = "eth0"               # 网络接口
    
    # 姿态序列（使用姿态名称）
    POSE_SEQUENCE = [
        "open_1",
        "open_2",
        "open_3",
    ]
    
    SPEED_FACTOR = 1.0               # 运动速度 (0.1-1.0)
    PAUSE_TIME = 0.0                 # 最后姿态停留时间（秒）
    # ==========================================
    
    print("="*70)
    print("🎬 灵巧手姿态序列控制器（非交互式）")
    print("="*70)
    print(f"🖐️  手: {HAND.upper()}")
    print(f"🌐 接口: {INTERFACE}")
    print("="*70)
    
    controller = SimpleHandSequence(hand=HAND, interface=INTERFACE)
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        # 执行序列
        success = controller.run_sequence(
            sequence=POSE_SEQUENCE,
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