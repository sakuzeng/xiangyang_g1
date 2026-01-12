#!/usr/bin/env python3
"""
test_phone_action.py
====================
测试模块2: 单独的动作执行序列
流程:
1. 初始化机械臂
2. 读取人工采集的IK数据 (MANUAL_IK_DATA)
3. 执行: 预备 -> 移动到目标 -> 手腕摆动 -> 撤退
"""

import sys
import os
import time
import json
import requests
from pathlib import Path

# 添加路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 导入依赖
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# 导入动作执行模块
try:
    from phone_touch_task import PhoneTouchController, RobotControlError, SafetyLimitError
    from common.robot_state_manager import robot_state
except ImportError:
    print("❌ 无法导入 phone_touch_task 或 robot_state，请检查路径")
    sys.exit(1)

# ==================== 配置 ====================
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"

# 人工采集的IK数据 (!!!请在此处填入您采集到的真实数据!!!)
# 格式: (joint_angles_list, torso_coord_tuple)
# joint_angles: [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw]
# torso_coord: (x, y, z)
MANUAL_IK_DATA = (
    [-0.576536, 0.256975, -0.006111, 0.711639, 1.315164, -0.042154, 0.251519],  # 示例关节角度
    (0.337,0.267,-0.178)                       # 示例目标坐标
)

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

            with robot_state.safe_arm_control(arm="left", source="manual_test", timeout=180.0):
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
                try:
                    payload = {"speak_msg": "财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)", "volume": 100, "source": "test_action"}
                    headers = {"Content-Type": "application/json"}
                    print(f"🔊 播报: 财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)")
                    requests.post(TTS_SERVER_URL, json=payload, headers=headers, timeout=1.0)
                except Exception as e:
                    print(f"⚠️ 语音播报失败: {e}")

                # 步骤7: 撤退
                print(f"\n【步骤7】撤退")
                retreat_sequence = ["phone_pre_final", "phone_pre_3", "phone_pre_2", "phone_pre_1"]
                for pose in retreat_sequence:
                    self.move_arm_to_pose(pose)
                    
            print("✨ 动作任务完成")
            
        except Exception as e:
            print(f"❌ 动作执行失败: {e}")
            if self.arm_client:
                print("⚠️ 尝试恢复到安全位置...")
                try:
                    self.move_arm_to_pose("phone_pre_1")
                except:
                    pass

def main():
    if len(sys.argv) < 2:
        interface = "eth0"
    else:
        interface = sys.argv[1]
        
    print("🚀 启动动作序列测试")
    ChannelFactoryInitialize(0, interface)
    
    # 动作执行
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