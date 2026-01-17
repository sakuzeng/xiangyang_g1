#!/usr/bin/env python3
"""
G1 双臂+双手联合控制器 - 精简版
从保存文件中读取并组合执行动作序列
"""
import sys
import time
import json
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from xiangyang.loco.common.robot_state_manager import robot_state


class JointArmHandController:
    """双臂+双手联合控制器 - 精简版"""
    
    def __init__(self, interface: str = "eth0"):
        self.interface = interface
        self.arm_client = None
        self.left_hand = None
        self.right_hand = None
        
        # 保存文件路径
        self.save_dir = Path("./saved_poses")
        self.left_arm_file = self.save_dir / "left_arm_poses.json"
        self.right_arm_file = self.save_dir / "right_arm_poses.json"
        self.left_hand_file = self.save_dir / "left_hand_poses.json"
        self.right_hand_file = self.save_dir / "right_hand_poses.json"
    
    def initialize(self) -> bool:
        """初始化"""
        try:
            print("🔧 初始化双臂+双手...")
            ChannelFactoryInitialize(0, self.interface)
            
            # 初始化手臂
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            if not self.arm_client.initialize_arms():
                print("❌ 手臂初始化失败")
                return False
            
            # 初始化左手
            self.left_hand = robot_state.get_or_create_hand_client("left", self.interface)
            if not self.left_hand.initialize_hand():
                print("❌ 左手初始化失败")
                return False
            
            # 初始化右手
            self.right_hand = robot_state.get_or_create_hand_client("right", self.interface)
            if not self.right_hand.initialize_hand():
                print("❌ 右手初始化失败")
                return False
            
            print("✅ 初始化成功")
            return True
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def _load_poses_from_file(self, file_path: Path):
        """从文件加载姿态"""
        if not file_path.exists():
            return {}
        with open(file_path, 'r') as f:
            return json.load(f)
    
    def load_arm_pose(self, arm: str, pose_name: str, speed: float = 0.5) -> bool:
        """加载手臂姿态"""
        file_path = self.left_arm_file if arm == "left" else self.right_arm_file
        poses = self._load_poses_from_file(file_path)
        
        if pose_name not in poses:
            print(f"❌ {arm} 臂未找到姿态: {pose_name}")
            return False
        
        positions = poses[pose_name]['positions']
        
        # 构建完整的14DOF数组
        offset = 0 if arm == 'left' else 7
        full_positions = [0.0] * 14
        current = self.arm_client.get_current_joint_positions(timeout=2.0)
        if current:
            full_positions = current
        full_positions[offset:offset+7] = positions
        
        try:
            with robot_state.safe_arm_control(arm=arm, source="load_pose"):
                self.arm_client.set_joint_positions(full_positions, speed_factor=speed)
            print(f"✅ {arm} 臂姿态加载完成: {pose_name}")
            return True
        except Exception as e:
            print(f"❌ {arm} 臂加载失败: {e}")
            return False
    
    def load_hand_pose(self, hand: str, pose_name: str, speed: float = 0.5) -> bool:
        """加载手部姿态"""
        file_path = self.left_hand_file if hand == "left" else self.right_hand_file
        poses = self._load_poses_from_file(file_path)
        
        if pose_name not in poses:
            print(f"❌ {hand} 手未找到姿态: {pose_name}")
            return False
        
        positions = poses[pose_name]['positions']
        hand_client = self.left_hand if hand == "left" else self.right_hand
        
        try:
            with robot_state.safe_hand_control(hand=hand, source="load_pose"):
                hand_client.set_joint_positions(positions, speed_factor=speed)
            print(f"✅ {hand} 手姿态加载完成: {pose_name}")
            return True
        except Exception as e:
            print(f"❌ {hand} 手加载失败: {e}")
            return False
    
    def execute_sequence(self, sequence: list, speed: float = 1.0):
        """
        执行动作序列
        
        Args:
            sequence: 动作序列，格式:
                [
                    {'type': 'arm', 'side': 'left', 'pose': 'pose1'},
                    {'type': 'hand', 'side': 'right', 'pose': 'pose2'},
                    {'type': 'wait', 'duration': 1.0}
                ]
            speed: 速度因子
        """
        print(f"\n🎬 开始执行动作序列 (共{len(sequence)}步)...")
        
        for i, step in enumerate(sequence, 1):
            print(f"\n步骤 {i}/{len(sequence)}: ", end='')
            
            if step['type'] == 'arm':
                self.load_arm_pose(step['side'], step['pose'], speed)
            elif step['type'] == 'hand':
                self.load_hand_pose(step['side'], step['pose'], speed)
            elif step['type'] == 'wait':
                duration = step.get('duration', 1.0)
                print(f"等待 {duration}s...")
                time.sleep(duration)
            else:
                print(f"⚠️  未知动作类型: {step['type']}")
        
        print("\n✅ 动作序列执行完成")
    
    def list_all_poses(self):
        """列出所有保存的姿态"""
        print("\n" + "="*70)
        print("📚 所有保存的姿态")
        print("="*70)
        
        for arm in ['left', 'right']:
            file_path = self.left_arm_file if arm == 'left' else self.right_arm_file
            poses = self._load_poses_from_file(file_path)
            if poses:
                print(f"\n💪 {arm.upper()} 臂:")
                for name in poses.keys():
                    print(f"  • {name}")
        
        for hand in ['left', 'right']:
            file_path = self.left_hand_file if hand == 'left' else self.right_hand_file
            poses = self._load_poses_from_file(file_path)
            if poses:
                print(f"\n🖐️  {hand.upper()} 手:")
                for name in poses.keys():
                    print(f"  • {name}")
        
        print("="*70)
    
    def shutdown(self):
        """关闭"""
        if self.arm_client:
            self.arm_client.stop_control()
        if self.left_hand:
            self.left_hand.stop_control()
        if self.right_hand:
            self.right_hand.stop_control()
        robot_state.reset_all_states()
        print("✅ 已关闭")


def main():
    """主程序"""
    interface = "eth0"
    
    # 解析命令行参数
    import argparse
    parser = argparse.ArgumentParser(description='G1 双臂+双手联合控制器')
    parser.add_argument('--speed', type=float, default=0.5, help='速度因子 (默认: 0.5)')
    parser.add_argument('--interface', type=str, default='eth0', help='网络接口')
    parser.add_argument('--list', action='store_true', help='列出所有姿态')
    
    args = parser.parse_args()
    
    print("="*70)
    print("🎮 G1 双臂+双手联合控制器")
    print("="*70)
    print(f"🌐 接口: {args.interface}")
    print(f"⚡ 速度: {args.speed}x")
    print("="*70)
    
    controller = JointArmHandController(interface=args.interface)
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        if args.list:
            controller.list_all_poses()
            return
        
        # 交互模式
        while True:
            print("\n" + "="*70)
            print("📋 选择操作")
            print("="*70)
            print("1. 加载手臂姿态")
            print("2. 加载手部姿态")
            print("3. 列出所有姿态")
            print("q. 退出")
            print("="*70)
            
            choice = input("\n选择: ").strip()
            
            if choice == '1':
                side = input("选择手臂 (left/right): ").strip()
                controller.list_all_poses()
                pose = input("输入姿态名称: ").strip()
                speed = input("速度因子 (默认1.0): ").strip()
                speed = float(speed) if speed else 1.0
                controller.load_arm_pose(side, pose, speed)
            
            elif choice == '2':
                side = input("选择手 (left/right): ").strip()
                controller.list_all_poses()
                pose = input("输入姿态名称: ").strip()
                speed = input("速度因子 (默认1.0): ").strip()
                speed = float(speed) if speed else 1.0
                controller.load_hand_pose(side, pose, speed)
            
            elif choice == '3':
                controller.list_all_poses()
            
            elif choice.lower() == 'q':
                break
            
            else:
                print("⚠️  无效选择")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  收到中断信号")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()