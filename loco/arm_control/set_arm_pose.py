#!/usr/bin/env python3
"""
G1 手臂姿态加载器 - 精简版
只从保存文件中加载姿态
"""
import sys
import time
import json
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
from pathlib import Path
# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xiangyang.loco.common.robot_state_manager import robot_state


class ArmPoseLoader:
    """手臂姿态加载器 - 精简版"""
    
    def __init__(self, arm: str = "left", interface: str = "eth0"):
        self.arm = arm
        self.interface = interface
        self.arm_client = None
        self.save_file = Path("./saved_poses") / f"{arm}_arm_poses.json"
    
    def initialize(self) -> bool:
        """初始化"""
        try:
            print(f"🔧 初始化 {self.arm.upper()} 手臂...")
            ChannelFactoryInitialize(0, self.interface)
            
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            
            if not self.arm_client.initialize_arms():
                print("❌ 初始化失败")
                return False
            
            time.sleep(2)
            print("✅ 初始化成功")
            return True
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def load_pose(self, pose_name: str, speed: float = 1.0) -> bool:
        """
        加载保存的姿态
        
        Args:
            pose_name: 姿态名称
            speed: 速度因子 (0.1-2.0)
        """
        if not self.save_file.exists():
            print(f"❌ 未找到保存文件: {self.save_file}")
            return False
        
        with open(self.save_file, 'r') as f:
            poses = json.load(f)
        
        if pose_name not in poses:
            print(f"❌ 未找到姿态: {pose_name}")
            print(f"可用姿态: {list(poses.keys())}")
            return False
        
        pose_data = poses[pose_name]
        positions = pose_data['positions']
        timestamp = pose_data.get('timestamp', 'Unknown')
        
        print(f"\n📥 加载姿态: {pose_name}")
        print(f"   保存时间: {timestamp}")
        print(f"   速度: {speed}x")
        
        try:
            # 构建完整的14DOF数组
            offset = 0 if self.arm == 'left' else 7
            full_positions = [0.0] * 14
            
            # 读取当前位置
            current = self.arm_client.get_current_joint_positions(timeout=2.0)
            if current:
                full_positions = current
            
            # 设置目标手臂位置
            full_positions[offset:offset+7] = positions
            
            # 执行移动
            with robot_state.safe_arm_control(arm=self.arm, source="load_pose", timeout=15.0):
                self.arm_client.set_joint_positions(full_positions, speed_factor=speed)
                time.sleep(0.5)
            
            print("✅ 姿态加载完成")
            return True
        except RuntimeError as e:
            print(f"❌ 加载失败: {e}")
            return False
        except Exception as e:
            print(f"❌ 执行失败: {e}")
            return False
    
    def list_poses(self):
        """列出所有保存的姿态"""
        if not self.save_file.exists():
            print(f"⚠️  未找到保存文件: {self.save_file}")
            return
        
        with open(self.save_file, 'r') as f:
            poses = json.load(f)
        
        if not poses:
            print("⚠️  无保存的姿态")
            return
        
        print("\n" + "="*70)
        print(f"📚 {self.arm.upper()} 手臂保存的姿态")
        print("="*70)
        for name, data in poses.items():
            timestamp = data.get('timestamp', 'Unknown')
            print(f"  • {name:20s} - 保存于 {timestamp}")
        print("="*70)
    
    def shutdown(self):
        """关闭"""
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state(self.arm)
            print("✅ 已关闭")


def main():
    """主程序"""
    arm = "left"
    interface = "eth0"
    
    # 解析命令行参数
    import argparse
    parser = argparse.ArgumentParser(description='G1 手臂姿态加载器')
    parser.add_argument('arm', choices=['left', 'right', 'l', 'r'], 
                       help='手臂选择 (left/right)')
    parser.add_argument('--pose', type=str, help='姿态名称')
    parser.add_argument('--speed', type=float, default=1.0, 
                       help='速度因子 0.1-2.0 (默认: 1.0)')
    parser.add_argument('--list', action='store_true', help='列出所有姿态')
    parser.add_argument('--interface', type=str, default='eth0', help='网络接口')
    
    args = parser.parse_args()
    
    # 标准化手臂名称
    arm = 'left' if args.arm in ['l', 'left'] else 'right'
    
    print("="*70)
    print("🎮 G1 手臂姿态加载器")
    print("="*70)
    print(f"💪 手臂: {arm.upper()}")
    print(f"🌐 接口: {args.interface}")
    print("="*70)
    
    loader = ArmPoseLoader(arm=arm, interface=args.interface)
    
    try:
        if not loader.initialize():
            sys.exit(1)
        
        # 列出姿态
        if args.list:
            loader.list_poses()
            return
        
        # 加载姿态
        if args.pose:
            loader.load_pose(args.pose, speed=args.speed)
            return
        
        # 交互模式
        while True:
            loader.list_poses()
            
            print("\n输入姿态名称 (或 q 退出): ", end='')
            name = input().strip()
            
            if name.lower() == 'q':
                break
            
            print("速度因子 (0.1-2.0, 默认1.0): ", end='')
            speed_input = input().strip()
            speed = float(speed_input) if speed_input else 1.0
            
            loader.load_pose(name, speed=speed)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  收到中断信号")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        loader.shutdown()


if __name__ == "__main__":
    main()