#!/usr/bin/env python3
"""
G1迎宾演示 - 打招呼 + 语音 + 前进
功能：
- 执行打招呼序列
- 在 hello 姿态时播报语音（不等待）
- 释放手臂控制后进行语音识别
- 根据识别结果决定是否前进
"""
import sys
import os
import json
import time
import math
import traceback
from pathlib import Path
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.dds.odometry_client import OdometryClient

# 添加项目根目录到路径 (为了导入 xiangyang 包)
current_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 导入依赖模块
try:
    from xiangyang.loco.common.tts_client import TTSClient
    from xiangyang.loco.common.asr_client import ASRClient
    from xiangyang.loco.common.interaction_client import InteractionClient
    from xiangyang.loco.common.robot_state_manager import robot_state
except ImportError as e:
    print(f"❌ 导入模块失败: {e}")
    sys.exit(1)


class G1GreetingDemo:
    """G1迎宾演示 - 打招呼 + 语音 + 前进"""
    
    def __init__(self, interface="eth0"):
        self.interface = interface
        
        # 控制参数
        self.MOVE_DISTANCE = 0.9        # 移动距离(m)
        self.LINEAR_VELOCITY = 0.3      # 线速度(m/s)
        self.POSITION_TOLERANCE = 0.05  # 位置容差(m)
        
        # 客户端
        self.loco_client = None
        self.odom_client = None
        self.arm_client = None
        self.hand_client = None
        
        self.arm_side = "right"
        self.hand_side = "right"
        
        # 姿态文件
        self.arm_pose_file = Path(f"../arm_control/saved_poses/{self.arm_side}_arm_poses.json")
        self.hand_pose_file = Path(f"../dex3_control/saved_poses/{self.hand_side}_hand_poses.json")
        self.arm_poses = {}
        self.hand_poses = {}
        
        self.is_arm_hand_initialized = False
        
        # 打招呼序列
        self.HELLO_SEQUENCE = [
            {'type': 'arm', 'pose': 'hello1'},
            {'type': 'hand', 'pose': 'hello'},  # 🆕 在这一步播报语音
            {'type': 'arm', 'pose': 'hello2'},
            {'type': 'arm', 'pose': 'hello3'},
            {'type': 'arm', 'pose': 'hello2'},
            {'type': 'hand', 'pose': 'close'},
            {'type': 'arm', 'pose': 'nature'},
        ]
        
        # 🆕 语音识别关键词（任意两个字匹配即可）
        self.TRIGGER_KEYWORDS = ["进", "入", "值", "班", "模", "式"]
    
    def load_pose_files(self):
        """加载姿态文件"""
        try:
            with open(self.arm_pose_file, 'r') as f:
                self.arm_poses = json.load(f)
            with open(self.hand_pose_file, 'r') as f:
                self.hand_poses = json.load(f)
            
            # 验证序列姿态
            for step in self.HELLO_SEQUENCE:
                if step['type'] == 'arm' and step['pose'] not in self.arm_poses:
                    print(f"❌ 缺少手臂姿态: {step['pose']}")
                    return False
                if step['type'] == 'hand' and step['pose'] not in self.hand_poses:
                    print(f"❌ 缺少灵巧手姿态: {step['pose']}")
                    return False
            return True
        except Exception as e:
            print(f"❌ 加载姿态失败: {e}")
            return False
    
    def initialize(self):
        """初始化"""
        try:
            ChannelFactoryInitialize(0, self.interface)
            
            # 初始化里程计
            print("📡 初始化里程计...")
            self.odom_client = OdometryClient(
                interface=self.interface,
                use_high_freq=False,
                use_low_freq=True
            )
            if not self.odom_client.initialize():
                print("❌ 里程计初始化失败")
                return False
            time.sleep(0.5)
            
            # 加载姿态
            if not self.load_pose_files():
                return False
            
            # 初始化运动控制
            self.loco_client = LocoClient()
            self.loco_client.Init()
            
            # 创建手臂和手客户端
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            self.hand_client = robot_state.get_or_create_hand_client(
                hand=self.hand_side, 
                interface=self.interface
            )
            
            print("✅ 初始化完成\n")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            traceback.print_exc()
            return False
    
    def initialize_arm_and_hand(self):
        """延迟初始化手臂和手"""
        if self.is_arm_hand_initialized:
            return True
        
        try:
            print("🔧 初始化手臂和灵巧手...")
            
            with robot_state.safe_arm_control(arm=self.arm_side, source="greeting_init", timeout=30):
                if not self.arm_client.initialize_arms():
                    return False
            
            with robot_state.safe_hand_control(hand=self.hand_side, source="greeting_init", timeout=30):
                if not self.hand_client.initialize_hand():
                    return False
            
            self.is_arm_hand_initialized = True
            print("✅ 手臂和灵巧手初始化完成\n")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def hello_gesture_with_voice(self):
        """执行打招呼动作并在特定步骤播报语音"""
        if not self.is_arm_hand_initialized:
            if not self.initialize_arm_and_hand():
                return False
        
        print("👋 开始打招呼...")
        
        # 🆕 在开始前暂停唤醒检测
        InteractionClient.pause_wake(source="greeting")
        
        success = False
        try:
            with robot_state.safe_arm_control(arm=self.arm_side, source="greeting_hello", timeout=60):
                with robot_state.safe_hand_control(hand=self.hand_side, source="greeting_hello", timeout=60):
                    
                    for i, step in enumerate(self.HELLO_SEQUENCE, 1):
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
                        
                        # 🆕 在 hello 姿态时播报语音（不等待）
                        if step_type == 'hand' and pose_name == 'hello':
                            time.sleep(0.3)  # 等待姿态到位
                            TTSClient.speak("您好,我是小安", volume=100, wait=False, source="greeting")
                        
                        if i < len(self.HELLO_SEQUENCE):
                            time.sleep(0.3)
                
                print("✅ 打招呼完成\n")
                success = True
        
        except Exception as e:
            print(f"❌ 打招呼失败: {e}")
            traceback.print_exc()
        
        finally:
            # 🆕 释放手臂和手控制
            print("🔓 释放手臂和手控制")
            if self.arm_client:
                self.arm_client.stop_control()
                robot_state.reset_arm_state(self.arm_side)
            if self.hand_client:
                self.hand_client.stop_control()
                robot_state.reset_hand_state(self.hand_side)
            
            # 🆕 恢复唤醒检测
            time.sleep(2.0)  # 等待语音播放完成
            InteractionClient.resume_wake(source="greeting")
        
        return success
    
    def check_voice_command(self):
        """
        🆕 检查语音指令是否包含触发关键词
        
        Returns:
            bool: True 表示应该执行前进，False 表示取消
        """
        print("\n" + "="*70)
        print("🎤 开始语音识别 (10秒内)")
        print("="*70)
        
        # 调用 ASR 服务进行识别
        recognized_text = ASRClient.recognize_live(duration=10.0)
        
        if not recognized_text:
            print("⚠️ 未识别到有效语音")
            return self._ask_user_confirmation()
        
        print(f"📝 识别结果: [{recognized_text}]")
        
        # 检查是否包含至少两个关键词
        matched_keywords = [kw for kw in self.TRIGGER_KEYWORDS if kw in recognized_text]
        
        print(f"🔍 匹配关键词: {matched_keywords}")
        
        if len(matched_keywords) >= 2:
            print("✅ 检测到触发指令，准备执行前进")
            # 🆕 播报确认语音
            TTSClient.speak("好的", volume=100, wait=False, source="greeting")
            return True
        else:
            print("⚠️ 未检测到完整触发指令")
            return self._ask_user_confirmation()
    
    def _ask_user_confirmation(self):
        """
        🆕 请求用户手动确认
        
        Returns:
            bool: True 继续执行，False 取消
        """
        print("\n" + "-"*70)
        while True:
            user_input = input("⌨️  请输入 'y' 继续执行前进，'n' 取消: ").strip().lower()
            if user_input == 'y':
                print("✅ 用户确认，继续执行")
                return True
            elif user_input == 'n':
                print("❌ 用户取消")
                return False
            else:
                print("⚠️ 输入无效，请输入 y 或 n")
    
    def move_forward_precise(self, distance: float):
        """基于里程计的精确前进"""
        print(f"🚶 精确前进 {distance:.2f}m")
        
        # 获取起始位置
        start_pos = self.odom_client.get_current_position()
        start_x, start_y = start_pos[0], start_pos[1]
        
        target_distance = abs(distance)
        base_velocity = self.LINEAR_VELOCITY
        
        max_time = target_distance / self.LINEAR_VELOCITY + 5
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                # 获取当前位置
                curr_pos = self.odom_client.get_current_position()
                curr_x, curr_y = curr_pos[0], curr_pos[1]
                
                # 计算已移动距离
                moved = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
                remaining = target_distance - moved
                
                # 到达目标
                if remaining <= self.POSITION_TOLERANCE:
                    break
                
                # 自适应速度（接近目标时减速）
                if remaining < 0.2:  # 最后20cm减速
                    velocity = base_velocity * max(0.3, remaining / 0.2)
                else:
                    velocity = base_velocity
                
                # 发送移动指令
                self.loco_client.Move(vx=velocity, vy=0.0, vyaw=0.0, continous_move=True)
                time.sleep(0.05)  # 20Hz
            
            # 停止
            self.loco_client.StopMove()
            time.sleep(0.3)
            
            # 打印结果
            final_pos = self.odom_client.get_current_position()
            final_x, final_y = final_pos[0], final_pos[1]
            actual_dist = math.sqrt((final_x - start_x)**2 + (final_y - start_y)**2)
            error_cm = abs(target_distance - actual_dist) * 100
            print(f"✅ 目标={target_distance:.2f}m, 实际={actual_dist:.2f}m, 误差={error_cm:.1f}cm\n")
            
        except Exception as e:
            print(f"❌ 移动异常: {e}")
            traceback.print_exc()
            self.loco_client.StopMove()
    
    def cleanup(self):
        """清理资源"""
        if self.loco_client:
            self.loco_client.StopMove()
        
        # 手臂和手的释放已在 hello_gesture_with_voice 的 finally 中处理
        # 这里只做最终的安全检查
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state(self.arm_side)
        if self.hand_client:
            self.hand_client.stop_control()
            robot_state.reset_hand_state(self.hand_side)
        
        # 打印里程计统计
        if self.odom_client:
            self.odom_client.print_stats()
    
    def run_greeting_sequence(self):
        """执行迎宾序列"""
        print("="*70)
        print("🎉 开始迎宾演示")
        print("="*70 + "\n")
        
        try:
            # 1. 打招呼 + 语音（在 hello 姿态时播报）
            # 函数内部会自动释放手臂和手控制
            print("[1/3] 打招呼并播报")
            if not self.hello_gesture_with_voice():
                return
            
            # 2. 语音识别并判断
            print("\n[2/3] 语音识别判断")
            if not self.check_voice_command():
                print("\n⚠️ 演示取消")
                return
            
            # 3. 精确前进1米
            print("\n[3/3] 精确前进1米")
            self.move_forward_precise(self.MOVE_DISTANCE)
            
            print("\n" + "="*70)
            print("✅ 迎宾演示完成!")
            print("="*70)
            
        except KeyboardInterrupt:
            print("\n⚠️ 用户中断")
        except Exception as e:
            print(f"❌ 执行失败: {e}")
            traceback.print_exc()
        finally:
            self.cleanup()


def main():
    INTERFACE = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    demo = G1GreetingDemo(interface=INTERFACE)
    
    try:
        if demo.initialize():
            demo.run_greeting_sequence()
        else:
            print("❌ 初始化失败")
            sys.exit(1)
    except Exception as e:
        print(f"❌ 程序异常: {e}")
        demo.cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()