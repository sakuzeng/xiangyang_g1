#!/usr/bin/env python3
"""
integrated_emergency_demo.py
============================
集成演示：
1. 语音交互 (异常播报 -> 确认)
2. 移动控制 (后退 -> 右转 -> 前进 -> 左转 -> 前进)
3. 动作执行 (手机触摸，使用人工数据)
"""

import sys
import os
import time
import math
import socket
import tempfile
import numpy as np
import requests
import json
from pathlib import Path

# 添加路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../audio')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../motion_control')))

# 导入依赖
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.dds.odometry_client import OdometryClient

# 尝试导入音频模块
try:
    from audio_record import AudioRecorder, CHANNELS, SAMPLE_WIDTH, FRAME_RATE
except ImportError:
    print("❌ 无法导入 audio_record，请检查路径")
    sys.exit(1)

# 导入动作执行模块
try:
    from phone_touch_task import PhoneTouchController, RobotControlError, SafetyLimitError
    from common.robot_state_manager import robot_state
except ImportError:
    print("❌ 无法导入 phone_touch_task 或 robot_state")
    sys.exit(1)

# ==================== 配置 ====================
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"
TTS_MONITOR_URL = "http://192.168.77.103:28001/monitor"
ASR_SERVER_URL = "http://localhost:8003/asr"

# 人工采集的IK数据 (需要用户替换为真实数据)
# 格式: (joint_angles_list, torso_coord_tuple)
# joint_angles: [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw]
# torso_coord: (x, y, z)
MANUAL_IK_DATA = (
    [-0.576536, 0.256975, -0.006111, 0.711639, 1.315164, -0.042154, 0.251519],  # 示例关节角度
    (0.337,0.267,-0.178)                       # 示例目标坐标
)

# ==================== 语音模块 ====================
class TTSClient:
    """HTTP TTS 客户端"""
    @staticmethod
    def speak(text, volume=100, wait=True):
        if not text: return
        try:
            payload = {"speak_msg": text, "volume": volume, "source": "integrated"}
            headers = {"Content-Type": "application/json"}
            print(f"🔊 {text}")
            response = requests.post(TTS_SERVER_URL, json=payload, headers=headers, timeout=2.0)
            if response.status_code != 200:
                print(f"⚠️ TTS错误: {response.status_code}")
                return
            
            if wait:
                result = response.json()
                data = result.get('data')
                if data and isinstance(data, dict):
                    task_id = data.get('task_id')
                    if task_id:
                        TTSClient._wait_for_completion(task_id)
        except Exception as e:
            print(f"❌ TTS失败: {e}")

    @staticmethod
    def _wait_for_completion(task_id, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                response = requests.get(TTS_MONITOR_URL, timeout=2.0)
                if response.status_code == 200:
                    data = response.json()
                    active_task = data.get('active_task')
                    queue_length = data.get('queue_length', 0)
                    if not active_task and queue_length == 0:
                        time.sleep(0.5) 
                        return True
            except:
                pass
            time.sleep(0.1)
        return False

class ASRClient:
    """HTTP ASR 客户端"""
    @staticmethod
    def recognize(audio_data):
        if audio_data is None or len(audio_data) == 0: return ""
        try:
            import wave
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_file:
                tmp_path = tmp_file.name
                with wave.open(tmp_path, 'wb') as wav_file:
                    wav_file.setnchannels(CHANNELS)
                    wav_file.setsampwidth(SAMPLE_WIDTH)
                    wav_file.setframerate(FRAME_RATE)
                    wav_file.writeframes(audio_data.tobytes())
            
            with open(tmp_path, 'rb') as f:
                files = {'file': (f'audio.wav', f, 'audio/wav')}
                response = requests.post(ASR_SERVER_URL, files=files, timeout=10.0)
            os.remove(tmp_path)
            
            if response.status_code == 200:
                return response.json().get('text', '').strip()
            return ""
        except Exception as e:
            print(f"❌ ASR失败: {e}")
            return ""

class VoiceInteraction(AudioRecorder):
    def __init__(self, interface_name="eth0"):
        super().__init__(interface_name)
        self.setup_audio_receiver()
        
    def listen_for_seconds(self, duration=6.0):
        print(f"\n👂 开始监听 ({duration}秒)...")
        buffer = []
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                data, _ = self.socket.recvfrom(2048)
                audio_np = np.frombuffer(data, dtype=np.int16)
                buffer.extend(audio_np)
            except socket.timeout:
                continue
            except Exception:
                break
        print("⏹️ 监听结束")
        return np.array(buffer)

    def run(self):
        try:
            # 1. 播报异常
            TTSClient.speak("财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)", wait=False)
            
            # 2. 询问
            TTSClient.speak("是否需要拨打对应变电站电话", wait=False)
            
            # # 3. 监听
            # audio_data = self.listen_for_seconds(3.0)
            # if len(audio_data) == 0:
            #     print("⚠️ 未采集到音频数据")
            #     TTSClient.speak("未检测到语音，操作取消", wait=True)
            #     return False

            # # 4. 识别
            # print("🤔 正在识别...")
            # text = ASRClient.recognize(audio_data)
            # print(f"📝 识别结果: [{text}]")
            
            # keywords = ["需要", "是", "拨打", "确认", "好的","要","需","须","药"]
            # confirmed = any(k in text for k in keywords)
            
            # 插入动作序列
            print("\n💪 执行确认动作序列...")
            try:
                # 动态导入 FullBodyPoseSequence
                sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../arm2dex3_control')))
                from exec_dual_arm2dex3_sequence import FullBodyPoseSequence
                
                action_controller = FullBodyPoseSequence(interface=self.interface_name)
                if action_controller.initialize():
                    SEQUENCE = [
                        ("nature", "nature", "nature", "nature"), 
                        ("inte_up", "keep", "open_1", "nature"),
                        ("nature", "inte_up", "close", "hello"),
                        ("nature", "nature", "nature", "nature")
                    ]
                    action_controller.run_sequence(SEQUENCE, speed_factor=1.0, pause_time=1.0)
                else:
                    print("⚠️ 动作控制器初始化失败")
            except Exception as e:
                print(f"⚠️ 动作执行出错: {e}")
                import traceback
                traceback.print_exc()
            finally:
                # 务必关闭控制器以释放资源，确保手臂停止控制
                if 'action_controller' in locals():
                    action_controller.shutdown()
                    print("✅ 动作控制器已关闭 (手臂控制已释放)")

            # confirmed = False
            # time.sleep(10)
            # 等待用户输入确认
            user_input = input("请输入 'y' 确认执行下一步 (输入其他取消): ").strip().lower()
            confirmed = (user_input == 'y')
            if confirmed:
                print("✅ 用户确认")
                TTSClient.speak("收到", wait=True)
                return True
            else:
                print("❌ 用户取消")
                TTSClient.speak("未检测到语音，操作取消", wait=True)
                return False
        finally:
            self.socket.close()

# ==================== 移动模块 ====================
class LocomotionController:
    def __init__(self, interface="eth0"):
        self.interface = interface
        self.LINEAR_VELOCITY = 0.3
        self.ANGULAR_VELOCITY = 0.50
        self.POSITION_TOLERANCE = 0.05
        self.ANGLE_TOLERANCE = 0.08
        self.loco_client = None
        self.odom_client = None

    def initialize(self):
        try:
            print("📡 初始化里程计...")
            self.odom_client = OdometryClient(interface=self.interface, use_high_freq=False, use_low_freq=True)
            if not self.odom_client.initialize():
                print("❌ 里程计初始化失败")
                return False
            time.sleep(0.5)
            
            self.loco_client = LocoClient()
            self.loco_client.Init()
            print("✅ 移动控制初始化完成")
            return True
        except Exception as e:
            print(f"❌ 移动控制初始化失败: {e}")
            return False

    def move_distance(self, distance: float):
        direction = 1 if distance > 0 else -1
        target_distance = abs(distance)
        start_pos = self.odom_client.get_current_position()
        start_x, start_y = start_pos[0], start_pos[1]
        
        print(f"{'🚶 前进' if direction > 0 else '🚶 后退'} {target_distance:.2f}m")
        base_velocity = self.LINEAR_VELOCITY * direction
        max_time = target_distance / abs(self.LINEAR_VELOCITY) + 10
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                curr_pos = self.odom_client.get_current_position()
                curr_x, curr_y = curr_pos[0], curr_pos[1]
                moved = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
                remaining = target_distance - moved
                
                if remaining <= self.POSITION_TOLERANCE: break
                
                velocity = base_velocity * max(0.3, remaining / 0.2) if remaining < 0.2 else base_velocity
                self.loco_client.Move(vx=velocity, vy=0.0, vyaw=0.0, continous_move=True)
                time.sleep(0.05)
            self.loco_client.StopMove()
            time.sleep(0.3)
        except Exception as e:
            print(f"❌ 移动异常: {e}")
            self.loco_client.StopMove()

    def turn_90(self, is_left: bool, angle_deg: float = 90):
        """转向指定角度 (使用绝对角度差控制)"""
        target_angle = math.radians(angle_deg)
        print(f"🔄 {'左' if is_left else '右'}转 {angle_deg}°")
        
        start_yaw = self.odom_client.get_current_yaw()
        target_yaw_diff = target_angle if is_left else -target_angle
        
        omega = self.ANGULAR_VELOCITY
        max_time = target_angle / self.ANGULAR_VELOCITY + 10
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                curr_yaw = self.odom_client.get_current_yaw()
                
                # 计算当前相对于起始点的角度变化 (归一化处理)
                current_diff = curr_yaw - start_yaw
                current_diff = math.atan2(math.sin(current_diff), math.cos(current_diff))
                
                # 计算剩余需要转过的角度
                remaining = target_yaw_diff - current_diff
                remaining = math.atan2(math.sin(remaining), math.cos(remaining))
                
                # 检查是否到达目标 (允许误差)
                if abs(remaining) <= self.ANGLE_TOLERANCE:
                    break
                
                # 自适应角速度（接近目标时减速）
                rot_direction = 1.0 if remaining > 0 else -1.0
                
                if abs(remaining) < math.radians(30):
                    scale = max(0.6, abs(remaining) / math.radians(30))
                    current_omega = omega * scale * rot_direction
                else:
                    current_omega = omega * rot_direction
                
                self.loco_client.Move(vx=0.0, vy=0.0, vyaw=current_omega, continous_move=True)
                time.sleep(0.05)
            
            self.loco_client.StopMove()
            time.sleep(0.8)
            
            # 结果验证
            final_yaw = self.odom_client.get_current_yaw()
            final_delta = final_yaw - start_yaw
            final_delta = math.atan2(math.sin(final_delta), math.cos(final_delta))
            error_deg = math.degrees(abs(target_yaw_diff - final_delta))
            print(f"✅ 转向完成: 实际转过 {math.degrees(final_delta):.1f}°, 误差 {error_deg:.1f}°")

            
        except Exception as e:
            print(f"❌ 旋转异常: {e}")
            self.loco_client.StopMove()

    def run_sequence(self):
        print("\n🚀 开始移动序列")
        # 1. 向后移动 0.9m
        # self.move_distance(-0.9)
        # 2. 向右转 85度
        self.turn_90(is_left=False, angle_deg=85)
        # 3. 前进 3m
        self.move_distance(2.2)
        # 4. 向左转 90度
        self.turn_90(is_left=True, angle_deg=90)
        # 5. 前进 0.6m
        self.move_distance(0.8)
        
        TTSClient.speak("马上为您拨通，请稍候", wait=False)

        print("✨ 移动序列完成")

# ==================== 动作模块 ====================
class ManualActionController(PhoneTouchController):
    def execute_with_manual_data(self, manual_data):
        print("\n" + "="*70)
        print("🎯 开始执行人工数据动作任务")
        print("="*70)
        
        try:
            self.target_joint_angles, self.target_torso_coord = manual_data
            
            print(f"📍 目标 Torso 坐标: {self.target_torso_coord}")
            print(f"🔧 目标关节角度: {self.target_joint_angles}")

            with robot_state.safe_arm_control(arm="left", source="integrated_demo", timeout=180.0):
                # 步骤1: 预备姿态
                print(f"\n【步骤1】执行预备姿态序列")
                prepare_sequence = ["phone_pre_1", "phone_pre_2", "phone_pre_3", "phone_pre_final"]
                for pose in prepare_sequence:
                    if not self.move_arm_to_pose(pose): raise RobotControlError(f"移动到预备姿态失败: {pose}")
                
                # 步骤2: 灵巧手
                print(f"\n【步骤2】设置灵巧手姿态")
                if not self.move_hand_to_pose("phone_pre_1"): raise RobotControlError("移动灵巧手失败")

                # 步骤3: 移动到目标
                print(f"\n【步骤3】移动到目标位置")
                if not self.move_arm_to_angles(self.target_joint_angles): raise RobotControlError("移动到目标位置失败")
                
                # 步骤4: 动作 (摆动)
                print(f"\n【步骤4】执行动作(摆动)")
                WRIST_YAW_INDEX = 6
                self.adjust_single_joint(WRIST_YAW_INDEX, self.wrist_pitch)
                self.adjust_single_joint(WRIST_YAW_INDEX, -self.wrist_pitch)
                
                # 步骤5: 设置灵巧手恢复原位
                print(f"\n【步骤5】设置灵巧手恢复原位")
                if not self.move_hand_to_pose("close"):
                    raise RobotControlError("灵巧手复位失败")

                # 步骤6: 肘关节收缩
                print(f"\n【步骤6】肘关节收缩")
                ELBOW_INDEX = 3
                print("  💪 收缩 -0.5 rad")
                self.adjust_single_joint(ELBOW_INDEX, -0.5)

                # 播报完成信息
                TTSClient.speak("财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)", wait=False)
                
                # 步骤7: 撤退
                print(f"\n【步骤7】撤退")
                retreat_sequence = ["phone_pre_final", "phone_pre_3", "phone_pre_2", "phone_pre_1"]
                for pose in retreat_sequence:
                    self.move_arm_to_pose(pose)
                    
            print("✨ 动作任务完成")
            
        except Exception as e:
            print(f"❌ 动作执行失败: {e}")
            if self.arm_client:
                self.move_arm_to_pose("phone_pre_1")

# ==================== 主入口 ====================
def main():
    if len(sys.argv) < 2:
        interface = "eth0"
    else:
        interface = sys.argv[1]
        
    print("🚀 启动集成演示程序")
    ChannelFactoryInitialize(0, interface)
    
    # 1. 语音交互
    voice = VoiceInteraction(interface)
    if not voice.run():
        return
        
    # 2. 移动
    mover = LocomotionController(interface)
    if not mover.initialize():
        return
    mover.run_sequence()
    
    # 3. 动作
    # 默认使用常规运控模式参数，如需修改请在此调整
    action = ManualActionController(
        interface=interface,
        expected_torso_z=-0.15,
        wrist_pitch=-0.60
    )
    if not action.initialize():
        return
        
    try:
        action.execute_with_manual_data(MANUAL_IK_DATA)
    finally:
        action.shutdown()

if __name__ == "__main__":
    main()