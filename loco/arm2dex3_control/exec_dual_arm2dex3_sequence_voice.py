#!/usr/bin/env python3
"""
带语音播报的全身协同动作序列控制器
功能：
- 同时控制左右手臂和左右灵巧手
- 支持语音播报
- 序列定义: (左臂, 右臂, 左手, 右手, 语音文本)
"""
import sys
import time
import json
import threading
import os
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Union

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from pathlib import Path
# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 导入依赖模块
try:
    from xiangyang.loco.common.tts_client import TTSClient
    from xiangyang.loco.common.robot_state_manager import robot_state
    from unitree_sdk2py.arm.arm_client import G1ArmGestures
except ImportError as e:
    print(f"❌ 导入模块失败: {e}")
    sys.exit(1)


class FullBodyPoseSequence:
    def __init__(self, interface: str = "eth0"):
        self.interface = interface
        self.arm_client = None
        self.left_hand_client = None
        self.right_hand_client = None
        
        self.left_arm_poses = {}
        self.right_arm_poses = {}
        self.left_hand_poses = {}
        self.right_hand_poses = {}
        
        # 获取当前脚本所在目录
        self.base_dir = Path(os.path.dirname(os.path.abspath(__file__)))
        
        # 相对路径指向 arm_control 和 dex3_control 的 saved_poses
        self.arm_pose_dir = self.base_dir.parent / "arm_control" / "saved_poses"
        self.hand_pose_dir = self.base_dir.parent / "dex3_control" / "saved_poses"
        
    def initialize(self) -> bool:
        """初始化机器人和加载姿态"""
        try:
            print("🔧 初始化全身控制器 (双臂 + 双手)...")
            ChannelFactoryInitialize(0, self.interface)
            
            # 1. 初始化双臂
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            if not self.arm_client.initialize_arms():
                print("❌ 双臂初始化失败")
                return False
                
            # 2. 初始化双手
            self.left_hand_client = robot_state.get_or_create_hand_client("left", self.interface)
            if not self.left_hand_client.initialize_hand():
                print("❌ 左手初始化失败")
                return False
                
            self.right_hand_client = robot_state.get_or_create_hand_client("right", self.interface)
            if not self.right_hand_client.initialize_hand():
                print("❌ 右手初始化失败")
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
        """加载姿态文件"""
        # 加载手臂姿态
        left_arm_file = self.arm_pose_dir / "left_arm_poses.json"
        right_arm_file = self.arm_pose_dir / "right_arm_poses.json"
        
        if left_arm_file.exists():
            with open(left_arm_file, 'r', encoding='utf-8') as f:
                self.left_arm_poses = json.load(f)
            print(f"📥 已加载左臂姿态: {len(self.left_arm_poses)} 个")
        else:
            print(f"⚠️  未找到左臂姿态文件: {left_arm_file}")
            
        if right_arm_file.exists():
            with open(right_arm_file, 'r', encoding='utf-8') as f:
                self.right_arm_poses = json.load(f)
            print(f"📥 已加载右臂姿态: {len(self.right_arm_poses)} 个")
        else:
            print(f"⚠️  未找到右臂姿态文件: {right_arm_file}")
            
        # 加载灵巧手姿态
        left_hand_file = self.hand_pose_dir / "left_hand_poses.json"
        right_hand_file = self.hand_pose_dir / "right_hand_poses.json"
        
        if left_hand_file.exists():
            with open(left_hand_file, 'r', encoding='utf-8') as f:
                self.left_hand_poses = json.load(f)
            print(f"📥 已加载左手姿态: {len(self.left_hand_poses)} 个")
        else:
            print(f"⚠️  未找到左手姿态文件: {left_hand_file}")
            
        if right_hand_file.exists():
            with open(right_hand_file, 'r', encoding='utf-8') as f:
                self.right_hand_poses = json.load(f)
            print(f"📥 已加载右手姿态: {len(self.right_hand_poses)} 个")
        else:
            print(f"⚠️  未找到右手姿态文件: {right_hand_file}")

    def _get_arm_positions(self, arm: str, pose_name: Optional[str], current_full_positions: List[float]) -> List[float]:
        """获取单臂关节目标位置"""
        offset = 0 if arm == 'left' else 7
        current_arm_positions = current_full_positions[offset:offset+7]
        
        # 1. 保持当前位置
        if pose_name is None or pose_name.lower() == "keep":
            return current_arm_positions
            
        # 2. 自然位
        if pose_name.lower() == "nature":
            nature_full = G1ArmGestures.get_pose("nature")
            return nature_full[offset:offset+7]
            
        # 3. 从文件加载
        poses_dict = self.left_arm_poses if arm == 'left' else self.right_arm_poses
        if pose_name in poses_dict:
            return poses_dict[pose_name]['positions']
            
        print(f"⚠️  警告: 未找到{arm}臂姿态 '{pose_name}'，保持当前位置")
        return current_arm_positions

    def _get_hand_positions(self, hand: str, pose_name: Optional[str], client) -> List[float]:
        """获取单手关节目标位置"""
        current_positions = client._current_jpos_des.copy()
        
        # 1. 保持当前位置
        if pose_name is None or pose_name.lower() == "keep":
            return current_positions
            
        # 2. 自然位 (使用 Dex3Client 内置的 nature_pos)
        if pose_name.lower() == "nature":
            return client._nature_pos
            
        # 3. 从文件加载
        poses_dict = self.left_hand_poses if hand == 'left' else self.right_hand_poses
        if pose_name in poses_dict:
            return poses_dict[pose_name]['positions']
            
        print(f"⚠️  警告: 未找到{hand}手姿态 '{pose_name}'，保持当前位置")
        return current_positions

    def execute_full_pose(self, left_arm: str, right_arm: str, left_hand: str, right_hand: str, speed_factor: float = 1.0) -> bool:
        """执行全身姿态 (双臂 + 双手)"""
        # --- 准备手臂目标 ---
        current_arm_pos = self.arm_client._current_jpos_des.copy()
        left_arm_target = self._get_arm_positions('left', left_arm, current_arm_pos)
        right_arm_target = self._get_arm_positions('right', right_arm, current_arm_pos)
        target_arm_positions = left_arm_target + right_arm_target
        
        if len(target_arm_positions) != 14:
            print(f"❌ 错误: 手臂目标关节数量不正确 ({len(target_arm_positions)})")
            return False

        # --- 准备手部目标 ---
        target_left_hand_pos = self._get_hand_positions('left', left_hand, self.left_hand_client)
        target_right_hand_pos = self._get_hand_positions('right', right_hand, self.right_hand_client)

        print(f"  ▶️  执行: L_Arm[{left_arm or 'keep'}] R_Arm[{right_arm or 'keep'}] | L_Hand[{left_hand or 'keep'}] R_Hand[{right_hand or 'keep'}]")
        
        # --- 并行执行 ---
        errors = []

        def run_arm():
            try:
                self.arm_client.set_joint_positions(target_arm_positions, speed_factor=speed_factor)
            except Exception as e:
                errors.append(f"Arm error: {e}")

        def run_left_hand():
            try:
                self.left_hand_client.set_joint_positions(target_left_hand_pos, speed_factor=speed_factor)
            except Exception as e:
                errors.append(f"Left Hand error: {e}")

        def run_right_hand():
            try:
                self.right_hand_client.set_joint_positions(target_right_hand_pos, speed_factor=speed_factor)
            except Exception as e:
                errors.append(f"Right Hand error: {e}")

        # 启动线程
        t1 = threading.Thread(target=run_arm)
        t2 = threading.Thread(target=run_left_hand)
        t3 = threading.Thread(target=run_right_hand)
        
        threads = [t1, t2, t3]
        for t in threads: t.start()
        for t in threads: t.join()
        
        if errors:
            print(f"  ❌ 执行出错: {'; '.join(errors)}")
            return False
            
        return True

    def run_sequence(self, sequence: List[Tuple[str, str, str, str, str]], speed_factor: float = 1.0, pause_time: float = 1.0):
        """执行动作序列"""
        print("\n" + "="*70)
        print(f"🎬 开始执行全身序列 ({len(sequence)} 步)")
        print("="*70)
        
        try:
            # 使用嵌套上下文管理器获取所有权限
            with robot_state.safe_dual_arm_control(source="full_sequence", timeout=120.0):
                with robot_state.safe_hand_control(hand="left", source="full_sequence", timeout=120.0):
                    with robot_state.safe_hand_control(hand="right", source="full_sequence", timeout=120.0):
                        
                        for i, (l_arm, r_arm, l_hand, r_hand, text) in enumerate(sequence, 1):
                            print(f"[{i}/{len(sequence)}]", end=" ")
                            
                            # 🗣️ 语音播报 (不等待)
                            if text:
                                TTSClient.speak(text, volume=100, wait=False, source="interaction")
                            
                            # 💪 执行动作
                            if not self.execute_full_pose(l_arm, r_arm, l_hand, r_hand, speed_factor):
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
            import traceback
            traceback.print_exc()
            return False

    def shutdown(self):
        print("\n🔧 停止所有控制...")
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state("left")
            robot_state.reset_arm_state("right")
        if self.left_hand_client:
            self.left_hand_client.stop_control()
            robot_state.reset_hand_state("left")
        if self.right_hand_client:
            self.right_hand_client.stop_control()
            robot_state.reset_hand_state("right")


def main():
    # ========== 📝 配置区域 ==========
    INTERFACE = "eth0"
    SPEED_FACTOR = 1.0
    PAUSE_TIME = 2.0  # 增加停留时间给语音一点空隙
    
    # 定义三个动作序列 [(左臂, 右臂, 左手, 右手, 语音文本)]
    
    # 序列 1: 左臂展示
    SEQUENCE_1 = [
        ("nature", "nature", "nature", "nature", ""), 
        ("inte_up", "keep", "open_1", "nature", "我在，有什么我可以帮您"),
        ("nature", "nature", "nature", "nature", "")
    ]
    
    # 序列 2: 右臂展示
    SEQUENCE_2 = [
        ("nature", "nature", "nature", "nature", ""), 
        ("inte_up", "keep", "open_1", "nature", "今日牛首变幺两号主变有公值为十五点六兆瓦，无重过载情况"),
        ("nature", "inte_up", "close", "hello", ""),
        ("nature", "nature", "nature", "nature", "")
    ]
    
    # 序列 3: 双臂协同
    SEQUENCE_3 = [
        ("nature", "nature", "nature", "nature", ""), 
        ("keep", "inte_up", "keep", "hello", "不客气"),
        ("nature", "nature", "nature", "nature", "")
    ]
    
    ALL_SEQUENCES = [SEQUENCE_1, SEQUENCE_2, SEQUENCE_3]
    # ================================
    
    controller = FullBodyPoseSequence(interface=INTERFACE)
    
    if not controller.initialize():
        sys.exit(1)
        
    try:
        # 循环执行三个序列
        for i, seq in enumerate(ALL_SEQUENCES, 1):
            print(f"\n" + "-"*30)
            print(f"⏳ 准备就绪: 序列 {i} / 3")
            
            while True:
                user_input = input(f"⌨️  请输入 'y' 开始执行序列 {i} (或 'q' 退出): ").strip().lower()
                if user_input == 'y':
                    print(f"🚀 开始执行序列 {i}...")
                    if not controller.run_sequence(seq, speed_factor=SPEED_FACTOR, pause_time=PAUSE_TIME):
                        print(f"⚠️ 序列 {i} 执行中断")
                    break
                elif user_input == 'q':
                    print("👋 用户取消，退出程序")
                    return
                else:
                    print("❌ 输入无效")

    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()