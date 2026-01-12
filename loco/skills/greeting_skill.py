import os
import sys
import json
import time
import traceback
from pathlib import Path
# 添加项目根目录到 sys.path 以支持绝对导入
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)  # 使用 insert(0) 确保优先加载本地项目代码

from xiangyang.loco.common import TTSClient
from xiangyang.loco.common import robot_state

class GreetingSkill:
    """
    迎宾技能
    功能：管理手臂/灵巧手连接，加载姿态，执行打招呼序列
    """
    def __init__(self, interface="eth0", arm_side="right", hand_side="right"):
        self.interface = interface
        self.arm_side = arm_side
        self.hand_side = hand_side
        
        self.arm_client = None
        self.hand_client = None
        self.is_initialized = False
        
        self.arm_poses = {}
        self.hand_poses = {}
        
        # 定义动作序列
        self.HELLO_SEQUENCE = [
            {'type': 'arm', 'pose': 'hello1'},
            {'type': 'hand', 'pose': 'hello'},
            {'type': 'arm', 'pose': 'hello2'},
            {'type': 'arm', 'pose': 'hello3'},
            {'type': 'arm', 'pose': 'hello2'},
            {'type': 'hand', 'pose': 'close'},
            {'type': 'arm', 'pose': 'nature'},
        ]

    def _load_pose_files(self):
        """加载姿态文件"""
        try:
            # 假设姿态文件相对于当前文件的位置
            # 注意：这里需要根据实际项目结构调整路径
            # 假设 skills 目录的上级是 loco，loco 下有 motion_control/arm_control 等
            base_dir = Path(__file__).parents[2] # xiangyang/
            
            # 尝试寻找路径，这里使用了相对于 xiangyang 包的路径假设
            arm_path = base_dir / f"loco/arm_control/saved_poses/{self.arm_side}_arm_poses.json"
            hand_path = base_dir / f"loco/dex3_control/saved_poses/{self.hand_side}_hand_poses.json"
            
            print(f"📂 加载姿态: {arm_path.name}")
            with open(arm_path, 'r') as f:
                self.arm_poses = json.load(f)
            with open(hand_path, 'r') as f:
                self.hand_poses = json.load(f)
            return True
        except Exception as e:
            print(f"❌ 加载姿态文件失败: {e}")
            return False

    def initialize(self):
        if self.is_initialized: return True
        
        try:
            if not self._load_pose_files(): return False
            
            print("🔧 初始化手臂和灵巧手...")
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            self.hand_client = robot_state.get_or_create_hand_client(self.hand_side, self.interface)
            
            with robot_state.safe_arm_control(arm=self.arm_side, source="greeting_init", timeout=30):
                if not self.arm_client.initialize_arms(): return False
            
            with robot_state.safe_hand_control(hand=self.hand_side, source="greeting_init", timeout=30):
                if not self.hand_client.initialize_hand(): return False
                
            self.is_initialized = True
            return True
        except Exception as e:
            print(f"❌ 技能初始化失败: {e}")
            return False

    def perform(self, voice_text, tts_source="greeting"):
        """执行打招呼并播报"""
        if not self.initialize(): return False
        
        print(f"👋 执行打招呼技能... 语音: {voice_text}")
        try:
            with robot_state.safe_arm_control(arm=self.arm_side, source="greeting_act", timeout=60):
                with robot_state.safe_hand_control(hand=self.hand_side, source="greeting_act", timeout=60):
                    
                    for step in self.HELLO_SEQUENCE:
                        step_type = step['type']
                        pose_name = step['pose']
                        
                        if step_type == 'arm':
                            positions = self.arm_poses[pose_name]['positions']
                            offset = 0 if self.arm_side == 'left' else 7
                            target = self.arm_client._current_jpos_des.copy()
                            target[offset:offset+7] = positions
                            self.arm_client.set_joint_positions(target, speed_factor=1.0)
                        
                        elif step_type == 'hand':
                            positions = self.hand_poses[pose_name]['positions']
                            self.hand_client.set_joint_positions(positions, speed_factor=1.0)
                        
                        # 触发语音
                        if step_type == 'hand' and pose_name == 'hello':
                            time.sleep(0.3)
                            TTSClient.speak(voice_text, volume=100, wait=False, source=tts_source)
                        
                        time.sleep(0.3)
            print("✅ 打招呼完成")
            return True
        except Exception as e:
            print(f"❌ 技能执行失败: {e}")
            traceback.print_exc()
            return False
        finally:
            self.stop() # 确保无论成功失败都释放控制权

    def stop(self):
        """释放控制权"""
        print("🔓 释放手臂/手控制")
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state(self.arm_side)
        if self.hand_client:
            self.hand_client.stop_control()
            robot_state.reset_hand_state(self.hand_side)