#!/usr/bin/env python3
"""
G1迎宾演示 V2 - 打招呼 + 语音 + 左转 + 前进 + 右转
功能：
- 执行打招呼序列
- 在 hello 姿态时播报语音（文本可配置）
- 向左转90度
- 向前走1.0米
- 向右转90度
- 停止
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
from pathlib import Path
# 添加项目根目录到路径 (为了导入 xiangyang 包)
# current_dir = os.path.dirname(__file__)
# project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 导入依赖模块
try:
    from xiangyang.loco.common.tts_client import TTSClient
    from xiangyang.loco.common.robot_state_manager import robot_state
except ImportError as e:
    print(f"❌ 导入模块失败: {e}")
    sys.exit(1)


class G1GreetingDemoV2:
    """G1迎宾演示 V2"""
    
    def __init__(self, voice_text, interface="eth0"):
        self.interface = interface
        self.voice_text = voice_text
        
        # 控制参数
        self.MOVE_DISTANCE = 0.9        # 移动距离(m)
        self.LINEAR_VELOCITY = 0.3      # 线速度(m/s)
        self.ANGULAR_VELOCITY = 0.50    # 角速度(rad/s)，28度/秒
        
        self.POSITION_TOLERANCE = 0.05  # 位置容差(m)
        self.ANGLE_TOLERANCE = 0.08     # 角度容差(rad)
        
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
    
    def load_pose_files(self):
        """加载姿态文件"""
        try:
            # 兼容处理：确保能找到文件
            # 如果当前工作目录不是脚本所在目录，相对路径可能会出错
            # 这里的 ../ 是相对于脚本位置的
            script_dir = Path(os.path.dirname(os.path.abspath(__file__)))
            
            # 构建绝对路径
            arm_file_path = (script_dir / self.arm_pose_file).resolve()
            hand_file_path = (script_dir / self.hand_pose_file).resolve()
            
            print(f"📂 加载姿态文件: {arm_file_path}")
            
            with open(arm_file_path, 'r') as f:
                self.arm_poses = json.load(f)
            with open(hand_file_path, 'r') as f:
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
            # 尝试使用原始相对路径再试一次 (如果上面解析失败)
            try:
                with open(self.arm_pose_file, 'r') as f:
                    self.arm_poses = json.load(f)
                with open(self.hand_pose_file, 'r') as f:
                    self.hand_poses = json.load(f)
                return True
            except:
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

    def safe_stop_arm_hand_before_move(self):
        """移动前停止手臂和手"""
        if robot_state.is_any_limb_controlling():
            print("🔓 移动前释放手臂和手控制")
            if self.arm_client:
                self.arm_client.stop_control()
                robot_state.reset_arm_state(self.arm_side)
            if self.hand_client:
                self.hand_client.stop_control()
                robot_state.reset_hand_state(self.hand_side)
            time.sleep(0.3)
    
    def hello_gesture_with_voice(self):
        """执行打招呼动作并在特定步骤播报语音"""
        if not self.is_arm_hand_initialized:
            if not self.initialize_arm_and_hand():
                return False
        
        print(f"👋 开始打招呼... 语音: {self.voice_text}")
        
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
                            TTSClient.speak(self.voice_text, volume=100, wait=False, source="greeting")
                        
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
            
            time.sleep(1.0) 
        
        return success
    
    def move_forward_precise(self, distance: float):
        """基于里程计的精确前进"""
        self.safe_stop_arm_hand_before_move()
        print(f"🚶 精确前进 {distance:.2f}m")
        
        # 获取起始位置
        start_pos = self.odom_client.get_current_position()
        start_x, start_y = start_pos[0], start_pos[1]
        
        target_distance = abs(distance)
        base_velocity = self.LINEAR_VELOCITY
        
        max_time = target_distance / self.LINEAR_VELOCITY + 10
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

    def turn_angle(self, angle_deg: float, direction: str):
        """
        基于里程计的精确旋转
        Args:
            angle_deg: 旋转角度(degree)
            direction: "left"或"right"
        """
        self.safe_stop_arm_hand_before_move()
        
        target_angle = math.radians(abs(angle_deg))
        
        # 获取起始Yaw角
        start_yaw = self.odom_client.get_current_yaw()
        
        # 计算旋转方向和目标累积角度
        sign = 1 if direction == "left" else -1
        target_delta = sign * target_angle
        
        print(f"🔄 {'左转' if direction == 'left' else '右转'} {math.degrees(target_angle):.1f}° (起始Yaw: {math.degrees(start_yaw):.1f}°)")
        
        max_time = target_angle / self.ANGULAR_VELOCITY + 10
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                # 获取当前Yaw
                curr_yaw = self.odom_client.get_current_yaw()
                
                # 计算当前相对于起始点的绝对角度变化 (归一化处理)
                current_diff = curr_yaw - start_yaw
                current_diff = math.atan2(math.sin(current_diff), math.cos(current_diff))
                
                # 计算剩余需要转过的角度
                remaining = target_delta - current_diff
                remaining = math.atan2(math.sin(remaining), math.cos(remaining))
                remaining_abs = abs(remaining)
                
                # 检查是否到达目标
                if remaining_abs <= self.ANGLE_TOLERANCE:
                    break
                
                # 过转保护
                if abs(current_diff) > target_angle * 1.2:
                    print(f"⚠️ 检测到过转 ({math.degrees(current_diff):.1f}°)，强制停止")
                    break
                
                # 自适应角速度
                rot_direction = 1.0 if remaining > 0 else -1.0
                
                if remaining_abs < math.radians(30):  # 最后30度减速
                    scale = max(0.6, remaining_abs / math.radians(30))
                    current_omega = self.ANGULAR_VELOCITY * scale * rot_direction
                else:
                    current_omega = self.ANGULAR_VELOCITY * rot_direction
                
                # 发送旋转指令
                self.loco_client.Move(vx=0.0, vy=0.0, vyaw=current_omega, continous_move=True)
                time.sleep(0.05)
            
            # 停止
            self.loco_client.StopMove()
            time.sleep(0.8)
            
            # 刷新里程计
            for _ in range(3):
                time.sleep(0.1)
                _ = self.odom_client.get_current_yaw()
            
            # 验证最终角度
            final_yaw = self.odom_client.get_current_yaw()
            actual_change = final_yaw - start_yaw
            actual_change = math.atan2(math.sin(actual_change), math.cos(actual_change))
            print(f"✅ 旋转完成。实际变化: {math.degrees(actual_change):.1f}°\n")
            
        except Exception as e:
            print(f"❌ 旋转异常: {e}")
            self.loco_client.StopMove()

    def cleanup(self):
        """清理资源"""
        if self.loco_client:
            self.loco_client.StopMove()
        
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state(self.arm_side)
        if self.hand_client:
            self.hand_client.stop_control()
            robot_state.reset_hand_state(self.hand_side)
        
        if self.odom_client:
            self.odom_client.print_stats()
    
    def run_greeting_sequence(self):
        """执行迎宾序列 V2"""
        print("="*70)
        print(f"🎉 开始迎宾演示 V2 - 语音: {self.voice_text}")
        print("="*70 + "\n")
        
        try:
            # 1. 打招呼 + 语音
            print("[1/4] 打招呼并播报")
            if not self.hello_gesture_with_voice():
                return
            
            # 2. 向左转90度
            print("\n[2/4] 向左转90度")
            self.turn_angle(90, "left")
            
            # 3. 向前走1.2米
            print("\n[3/4] 向前走1.0米")
            self.move_forward_precise(self.MOVE_DISTANCE)
            
            # 4. 向右转90度
            print("\n[4/4] 向右转90度")
            self.turn_angle(90, "right")
            
            print("\n" + "="*70)
            print("✅ 迎宾演示 V2 完成!")
            print("="*70)
            
        except KeyboardInterrupt:
            print("\n⚠️ 用户中断")
        except Exception as e:
            print(f"❌ 执行失败: {e}")
            traceback.print_exc()
        finally:
            self.cleanup()


def main():
    # 在此处指定语音文本
    VOICE_TEXT = "尊敬的各位领导，大家好，我是监控机器人小安，欢迎莅临江南集控站指导工作。"
    
    INTERFACE = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    demo = G1GreetingDemoV2(voice_text=VOICE_TEXT, interface=INTERFACE)
    
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