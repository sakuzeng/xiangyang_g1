import sys
import os
import json
import time
import math
import requests
import traceback
import threading
from pathlib import Path
from typing import List, Dict, Any
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.dds.odometry_client import OdometryClient, OdometryData
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from common.robot_state_manager import robot_state

# TTS 配置
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"
TTS_MONITOR_URL = "http://192.168.77.103:28001/monitor"

class TTSClient:
    """HTTP TTS 客户端 - 流式播放 + 等待完成"""
    
    @staticmethod
    def speak(text, volume=80, wait=True, source="integrated"):
        """发送TTS请求并可选等待播放完成"""
        if not text:
            return
        
        try:
            payload = {
                "speak_msg": text,
                "volume": volume,
                "source": source
            }
            headers = {"Content-Type": "application/json"}
            
            print(f"🔊 {text}")
            response = requests.post(TTS_SERVER_URL, json=payload, headers=headers, timeout=2.0)
            
            if response.status_code != 200:
                print(f"⚠️ TTS错误: {response.status_code}")
                return
            
            result = response.json()
            
            if result.get('msg') == 'ignored_filtered':
                print(f"⚠️ TTS被过滤")
                return
            
            data = result.get('data')
            if not data or not isinstance(data, dict):
                print(f"⚠️ TTS响应异常")
                return
            
            task_id = data.get('task_id')
            
            if wait and task_id:
                TTSClient._wait_for_completion(task_id)
                
        except requests.exceptions.RequestException as e:
            print(f"❌ TTS请求失败: {e}")
        except Exception as e:
            print(f"❌ TTS失败: {e}")

    @staticmethod
    def _wait_for_completion(task_id, timeout=30):
        """轮询监控接口，等待任务完成"""
        start_time = time.time()
        check_interval = 0.05
        
        task_started = False
        consecutive_empty_checks = 0
        
        while time.time() - start_time < timeout:
            try:
                response = requests.get(TTS_MONITOR_URL, timeout=2.0)
                
                if response.status_code == 200:
                    data = response.json()
                    active_task = data.get('active_task')
                    queue_length = data.get('queue_length', 0)
                    
                    # 检查任务是否正在播放
                    if active_task:
                        current_id = active_task.get('id')
                        if current_id == task_id:
                            task_started = True
                            consecutive_empty_checks = 0
                            time.sleep(check_interval)
                            continue
                    
                    # 任务开始后，需要连续3次检查都为空才认为完成
                    if task_started:
                        if not active_task and queue_length == 0:
                            consecutive_empty_checks += 1
                            
                            if consecutive_empty_checks >= 3:
                                time.sleep(0.2)
                                return True
                        else:
                            consecutive_empty_checks = 0
                    
            except:
                pass
                
            time.sleep(check_interval)
        
        return False


class G1PatrolDemo:
    """G1巡逻演示 - 基于里程计反馈的精确控制"""
    
    def __init__(self, interface="eth0", turn_direction="right"):
        self.interface = interface
        self.turn_direction = turn_direction.lower()
        
        # 控制参数
        self.MOVE_DISTANCE = 0.6        # 移动距离(m)
        self.LINEAR_VELOCITY = 0.3      # 线速度(m/s)
        self.ANGULAR_VELOCITY = 0.80    # 用户确认速度合适
        
        # 移除人为补偿，直接使用标准的 90 度
        self.TURN_ANGLE = math.pi / 2
        
        # 控制精度
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
            {'type': 'hand', 'pose': 'hello'},
            {'type': 'arm', 'pose': 'hello2'},
            {'type': 'arm', 'pose': 'hello3'},
            {'type': 'arm', 'pose': 'hello2'},
            {'type': 'hand', 'pose': 'close'},
            {'type': 'arm', 'pose': 'nature'},
        ]
    
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
            
            # 🆕 初始化里程计
            print("📡 初始化里程计...")
            self.odom_client = OdometryClient(
                interface=self.interface,
                use_high_freq=False,
                use_low_freq=True
            )
            if not self.odom_client.initialize():
                print("❌ 里程计初始化失败")
                return False
            
            # 等待接收第一帧数据
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
            
            with robot_state.safe_arm_control(arm=self.arm_side, source="init", timeout=30):
                if not self.arm_client.initialize_arms():
                    return False
            
            with robot_state.safe_hand_control(hand=self.hand_side, source="init", timeout=30):
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
            if self.arm_client:
                self.arm_client.stop_control()
                robot_state.reset_arm_state(self.arm_side)
            if self.hand_client:
                self.hand_client.stop_control()
                robot_state.reset_hand_state(self.hand_side)
            time.sleep(0.3)
    
    def move_distance(self, distance: float, direction: int = 1):
        """
        🆕 优化的基于里程计的精确移动（解决顿挫问题）
        
        Args:
            distance: 移动距离(m)
            direction: 1=前进, -1=后退
        """
        self.safe_stop_arm_hand_before_move()
        
        # 获取起始位置
        start_pos = self.odom_client.get_current_position()
        start_x, start_y = start_pos[0], start_pos[1]
        
        # 计算目标距离
        target_distance = abs(distance)
        base_velocity = self.LINEAR_VELOCITY * direction
        
        print(f"{'🚶 前进' if direction > 0 else '🚶 后退'} {target_distance:.2f}m")
        
        max_time = target_distance / abs(self.LINEAR_VELOCITY) + 5
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
                
                # 🆕 自适应速度（接近目标时减速）
                if remaining < 0.2:  # 最后20cm减速
                    velocity = base_velocity * max(0.3, remaining / 0.2)
                else:
                    velocity = base_velocity
                
                # 🔧 关键修改：使用 Move 方法替代 SetVelocity
                # Move 方法会自动设置 continous_move，避免频繁重新发送
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
            self.loco_client.StopMove()
    
    def turn_angle(self, angle: float, direction: str = None):
        """
        🆕 优化的基于里程计的精确旋转（修复跨越±180°边界问题）
        
        Args:
            angle: 旋转角度(rad)
            direction: "left"或"right"
        """
        self.safe_stop_arm_hand_before_move()
        
        direction = direction or self.turn_direction
        target_angle = abs(angle)
        
        # 获取起始Yaw角
        start_yaw = self.odom_client.get_current_yaw()
        
        # 🆕 计算旋转方向和目标累积角度
        sign = 1 if direction == "left" else -1
        target_delta = sign * target_angle
        
        print(f"🔄 {'左转' if direction == 'left' else '右转'} {math.degrees(target_angle):.1f}° (起始Yaw: {math.degrees(start_yaw):.1f}°)")
        
        # 🆕 使用绝对角度差控制，而非累积增量
        # omega = self.ANGULAR_VELOCITY if direction == "left" else -self.ANGULAR_VELOCITY
        
        max_time = target_angle / self.ANGULAR_VELOCITY + 8
        start_time = time.time()
        
        try:
            # === 第一阶段：主要旋转 ===
            while time.time() - start_time < max_time:
                # 获取当前Yaw
                curr_yaw = self.odom_client.get_current_yaw()
                
                # 计算当前相对于起始点的绝对角度变化 (归一化处理)
                current_diff = curr_yaw - start_yaw
                current_diff = math.atan2(math.sin(current_diff), math.cos(current_diff))
                
                # 计算剩余需要转过的角度 (注意符号)
                # target_delta 包含了方向信息 (左为正，右为负)
                remaining = target_delta - current_diff
                remaining = math.atan2(math.sin(remaining), math.cos(remaining))
                remaining_abs = abs(remaining)
                
                # 检查是否到达目标
                if remaining_abs <= self.ANGLE_TOLERANCE:
                    break
                
                # 🛡️ 过转保护：如果转过的角度明显超过目标（例如 > 120%），强制停止
                # 防止因惯性或控制滞后导致的“绕圈”现象
                if abs(current_diff) > target_angle * 1.2:
                    print(f"⚠️ 检测到过转 ({math.degrees(current_diff):.1f}°)，强制停止")
                    break
                
                # 🆕 自适应角速度（接近目标时减速）
                # 根据剩余角度的符号决定旋转方向，实现闭环修正
                rot_direction = 1.0 if remaining > 0 else -1.0
                
                if remaining_abs < math.radians(30):  # 最后30度减速
                    scale = max(0.4, remaining_abs / math.radians(30))
                    current_omega = self.ANGULAR_VELOCITY * scale * rot_direction
                else:
                    current_omega = self.ANGULAR_VELOCITY * rot_direction
                
                # 发送旋转指令
                self.loco_client.Move(vx=0.0, vy=0.0, vyaw=current_omega, continous_move=True)
                time.sleep(0.05)
            
            # 停止
            self.loco_client.StopMove()
            
            # 🔧 增加等待时间，确保机器人完全停稳且里程计数据更新
            time.sleep(0.8)  # 🆕 0.5 → 0.8秒
            
            # 🆕 等待期间多次读取，取最新值
            for _ in range(3):
                time.sleep(0.1)
                _ = self.odom_client.get_current_yaw()  # 触发数据更新
            
            # 验证最终角度
            final_yaw = self.odom_client.get_current_yaw()
            final_delta = final_yaw - start_yaw
            final_delta = math.atan2(math.sin(final_delta), math.cos(final_delta))
            error_deg = math.degrees(abs(target_delta - final_delta))
            
            print(f"✅ 第一阶段: 目标={math.degrees(target_delta):.1f}°, 实际={math.degrees(final_delta):.1f}°, 误差={error_deg:.1f}°")
            
            print()
            
        except Exception as e:
            print(f"❌ 旋转异常: {e}")
            traceback.print_exc()
            self.loco_client.StopMove()
    
    def hello_gesture(self):
        """执行打招呼动作"""
        if not self.is_arm_hand_initialized:
            if not self.initialize_arm_and_hand():
                return False
        
        print("👋 开始打招呼...")
        
        try:
            with robot_state.safe_arm_control(arm=self.arm_side, source="hello", timeout=60):
                with robot_state.safe_hand_control(hand=self.hand_side, source="hello", timeout=60):
                    
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
                        
                        if i < len(self.HELLO_SEQUENCE):
                            time.sleep(0.3)
                    
                    print("✅ 打招呼完成\n")
                    return True
        
        except Exception as e:
            print(f"❌ 打招呼失败: {e}")
            return False
    
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
        
        # 打印里程计统计
        if self.odom_client:
            self.odom_client.print_stats()
    
    def run_patrol_sequence(self):
        """执行巡逻序列"""
        print("="*70)
        print("🚀 开始巡逻 (基于里程计精确控制)")
        print("="*70 + "\n")
        
        try:
            # 1. 后退
            print("[1/9] 后退")
            self.move_distance(self.MOVE_DISTANCE, direction=-1)
            
            # 2. 转向
            print(f"[2/9] {self.turn_direction.upper()}转90°")
            self.turn_angle(self.TURN_ANGLE, direction=self.turn_direction)
            
            # 3. 前进
            print("[3/9] 前进")
            self.move_distance(self.MOVE_DISTANCE, direction=1)
            
            # 4. 语音播报
            print("[4/9] 语音播报")
            TTSClient.speak("您好,我是电网哨兵小安,有什么我可以帮您的", volume=80, wait=False)
            
            # 5. 打招呼
            print("[5/9] 打招呼")
            self.hello_gesture()
            
            # 6. 关闭手臂和手
            print("[6/9] 关闭手臂和手")
            if self.arm_client:
                self.arm_client.stop_control()
                robot_state.reset_arm_state(self.arm_side)
            if self.hand_client:
                self.hand_client.stop_control()
                robot_state.reset_hand_state(self.hand_side)
            time.sleep(0.3)
            
            # 7. 后退
            print("[7/9] 后退")
            self.move_distance(self.MOVE_DISTANCE, direction=-1)
            
            # 8. 反向转
            print("[8/9] 反向转90°")
            reverse = "left" if self.turn_direction == "right" else "right"
            self.turn_angle(self.TURN_ANGLE, direction=reverse)
            
            # 9. 前进回原位
            print("[9/9] 前进回原位")
            self.move_distance(self.MOVE_DISTANCE, direction=1)
            
            print("="*70)
            print("✅ 巡逻完成!")
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
    TURN_DIRECTION = "right"
    
    demo = G1PatrolDemo(interface=INTERFACE, turn_direction=TURN_DIRECTION)
    
    try:
        if demo.initialize():
            demo.run_patrol_sequence()
        else:
            print("❌ 初始化失败")
            sys.exit(1)
    except Exception as e:
        print(f"❌ 程序异常: {e}")
        demo.cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()