#!/usr/bin/env python3
"""
phone_touch_task.py
===================

手机触摸任务控制器

🆕 更新:
- 集成升级版 screen_to_ik (支持Torso Z验证)
- 深度获取采用中值填补策略
"""

import sys
import time
import json
from pathlib import Path
from typing import Optional, List, Tuple
import numpy as np

# SDK导入
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient  # 🆕 导入LocoClient
import os
# 添加项目根目录到sys.path以导入common模块
# current_dir = os.path.dirname(os.path.abspath(__file__))
# if current_dir not in sys.path:
#     sys.path.append(current_dir)
from pathlib import Path
# project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xiangyang.loco.common.robot_state_manager import robot_state
from xiangyang.loco.common.tts_client import TTSClient  # 🆕 导入TTSClient
from xiangyang.loco.common.logger import setup_logger

# 🆕 导入升级版求解器
from screen_to_ik import ScreenToIKSolver
# 🆕 导入自定义异常
from touch_exceptions import (
    TouchSystemError,
    RobotControlError,
    SafetyLimitError,
    IKSolutionError
)

logger = setup_logger("phone_touch_task")


class PhoneTouchController:
    """手机触摸任务控制器"""
    
    def __init__(self, 
                 interface: str = "eth0",
                 expected_torso_z: float = -0.17,     # 🆕 屏幕Z基准
                 torso_z_tolerance: float = 0.05,     # 🆕 Z容差
                 measurement_error: Optional[List[float]] = None, # 🆕 测量误差
                 wrist_pitch: float = -0.6,           # 🆕 手腕下倾角
                 torso_x_range: Optional[Tuple[float, float]] = None, # 🆕 X范围限制
                 torso_y_range: Optional[Tuple[float, float]] = None): # 🆕 Y范围限制
        """
        初始化控制器
        
        Args:
            interface: 网络接口
            expected_torso_z: 屏幕平面Torso Z基准值 (米)
            torso_z_tolerance: Z值容差 (米)
            measurement_error: 测量误差修正向量
            wrist_pitch: 手腕下倾角 (rad)
            torso_x_range: Torso X坐标允许范围 (min, max)
            torso_y_range: Torso Y坐标允许范围 (min, max)
        """
        self.interface = interface
        self.arm_client = None
        self.hand_client = None
        self.ik_solver = None
        
        # 🆕 保存参数
        self.expected_torso_z = expected_torso_z
        self.torso_z_tolerance = torso_z_tolerance
        self.measurement_error = measurement_error
        self.wrist_pitch = wrist_pitch
        self.torso_x_range = torso_x_range
        self.torso_y_range = torso_y_range

        
        # 姿态文件路径
        self.arm_pose_file = Path("../arm_control/saved_poses/left_arm_poses.json")
        self.hand_pose_file = Path("../dex3_control/saved_poses/left_hand_poses.json")
        self.arm_poses = {}
        self.hand_poses = {}
        
        # 任务状态
        self.emergency_exit = False
        self.target_joint_angles = None
        self.target_torso_coord = None
        
        # 安全阈值配置
        self.SAFE_X_THRESHOLD = 0.07
        self.SAFE_Z_THRESHOLD = -0.1
    
    def initialize(self) -> None:
        """
        初始化所有组件
        
        Raises:
            RobotControlError: 如果初始化失败
        """
        logger.info("\n" + "="*70)
        logger.info("🔧 初始化手机触摸控制器")
        logger.info("="*70)
        
        try:
            # 1. 初始化通道
            ChannelFactoryInitialize(0, self.interface)
            
            # 2. 初始化左臂
            logger.info("💪 初始化左臂...")
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            if not self.arm_client.initialize_arms():
                raise RobotControlError("左臂初始化失败")
            
            # 3. 初始化左灵巧手
            logger.info("✋ 初始化左灵巧手...")
            self.hand_client = robot_state.get_or_create_hand_client(
                hand="left",
                interface=self.interface
            )
            if not self.hand_client.initialize_hand():
                raise RobotControlError("左灵巧手初始化失败")
            
            # 4. 加载姿态文件
            logger.info("📂 加载姿态库...")
            self._load_poses()
            
            # 5. 🆕 初始化升级版IK求解器
            logger.info("🔧 初始化IK求解器...")
            self.ik_solver = ScreenToIKSolver(
                expected_torso_z=self.expected_torso_z,
                torso_z_tolerance=self.torso_z_tolerance,
                measurement_error=self.measurement_error
            )
            logger.info(f"   ✅ Torso Z基准: {self.expected_torso_z:.3f}m")
            logger.info(f"   ✅ 测量误差: {self.measurement_error}")
            logger.info(f"   ✅ 手腕下倾角: {self.wrist_pitch:.3f} rad")
            if self.torso_x_range:
                logger.info(f"   ✅ X范围限制: {self.torso_x_range}")
            if self.torso_y_range:
                logger.info(f"   ✅ Y范围限制: {self.torso_y_range}")
            
            logger.info("✅ 所有组件初始化成功\n")
            return True
            
        except Exception as e:
            logger.error(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            if isinstance(e, TouchSystemError):
                raise e
            raise RobotControlError(f"初始化过程发生未知错误: {e}")
    
    def _load_poses(self) -> None:
        """加载姿态库"""
        try:
            if not self.arm_pose_file.exists():
                raise FileNotFoundError(f"手臂姿态文件不存在: {self.arm_pose_file}")
            with open(self.arm_pose_file, 'r') as f:
                self.arm_poses = json.load(f)
            logger.info(f"   ✅ 手臂姿态: {len(self.arm_poses)} 个")
            
            if not self.hand_pose_file.exists():
                raise FileNotFoundError(f"灵巧手姿态文件不存在: {self.hand_pose_file}")
            with open(self.hand_pose_file, 'r') as f:
                self.hand_poses = json.load(f)
            logger.info(f"   ✅ 灵巧手姿态: {len(self.hand_poses)} 个")
            
        except Exception as e:
            raise RobotControlError(f"加载姿态失败: {e}")
    
    def _get_current_end_position(self) -> Optional[Tuple[float, float, float]]:
        """通过FK计算当前末端位置 (保持不变)"""
        try:
            current_joints = self.arm_client._current_jpos_des[0:7]
            full_state = [0.0] + list(current_joints) + [0.0]
            current_frame = self.ik_solver.chain.forward_kinematics(full_state)
            
            x = current_frame[0, 3]
            y = current_frame[1, 3]
            z = current_frame[2, 3]
            
            return (x, y, z)
            
        except Exception as e:
            logger.warning(f"⚠️  FK计算失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _check_need_lift(self) -> bool:
        """检查是否需要抬起手臂 (保持不变)"""
        pos = self._get_current_end_position()
        
        if pos is None:
            logger.warning("⚠️  无法获取末端位置,假定不需要抬起")
            return False
        
        x, y, z = pos
        
        logger.info(f"\n📍 当前末端位置 (Torso坐标系):")
        logger.info(f"   X = {x:+.4f} m")
        logger.info(f"   Y = {y:+.4f} m")
        logger.info(f"   Z = {z:+.4f} m")
        
        need_lift = (x > self.SAFE_X_THRESHOLD) and (z > self.SAFE_Z_THRESHOLD)
        
        if need_lift:
            logger.warning(f"🚨 手臂在桌面上! (x={x:.3f} > {self.SAFE_X_THRESHOLD}, z={z:.3f} > {self.SAFE_Z_THRESHOLD})")
        else:
            logger.info(f"✅ 手臂不在桌面上,可以直接关闭")
        
        return need_lift
    
    def _confirm_execution(self, target_index: int) -> bool:
        """显示目标信息并等待用户确认 (保持不变)"""
        logger.info("\n" + "="*70)
        logger.info("📋 任务确认信息")
        logger.info("="*70)
        logger.info(f"🎯 目标区域编号: {target_index}")
        logger.info(f"\n📍 Torso坐标系目标位置:")
        logger.info(f"   X = {self.target_torso_coord[0]:+.4f} m")
        logger.info(f"   Y = {self.target_torso_coord[1]:+.4f} m")
        logger.info(f"   Z = {self.target_torso_coord[2]:+.4f} m")
        
        logger.info(f"\n🔧 关节角度 (弧度):")
        joint_names = [
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"
        ]
        for i, (name, angle) in enumerate(zip(joint_names, self.target_joint_angles)):
            logger.info(f"   [{i}] {name:<20}: {angle:+.4f}")
        
        logger.info("\n" + "="*70)
        logger.info("📋 IK解算结果 (复制用)")
        logger.info("="*70)
        
        new_joints = self.target_joint_angles
        
        logger.info("\n# 紧凑格式(单行):")
        compact_str = "[" + ", ".join([f"{val:.6f}" for val in new_joints]) + "]"
        logger.info(f"new_joints = {compact_str}")
        
        logger.info("\n" + "="*70)
        logger.info("⚠️  请确认以上信息是否正确!")
        logger.info("="*70)
        
        while True:
            # input 保持不变，用于交互
            response = input("\n是否继续执行? (y/n): ").strip().lower()
            
            if response == 'y' or response == 'yes':
                logger.info("✅ 用户确认,开始执行任务...")
                return True
            elif response == 'n' or response == 'no':
                logger.info("❌ 用户取消,任务终止")
                return False
            else:
                logger.warning("⚠️  输入无效,请输入 y 或 n")
    
    # 🆕 后续方法保持不变
    def move_arm_to_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """移动手臂到指定姿态"""
        if pose_name not in self.arm_poses:
            logger.error(f"❌ 手臂姿态不存在: {pose_name}")
            return False
        
        positions = self.arm_poses[pose_name]['positions']
        target = self.arm_client._current_jpos_des.copy()
        target[0:7] = positions
        
        logger.info(f"  ▶️  移动手臂到: {pose_name}")
        try:
            self.arm_client.set_joint_positions(target, speed_factor=speed_factor)
            time.sleep(0.3)
            logger.info(f"  ✅ 完成")
            return True
        except Exception as e:
            logger.error(f"  ❌ 失败: {e}")
            return False
    
    def move_arm_to_angles(self, joint_angles: List[float], speed_factor: float = 1.0) -> bool:
        """移动手臂到指定关节角度"""
        target = self.arm_client._current_jpos_des.copy()
        target[0:7] = joint_angles
        
        logger.info(f"  ▶️  移动到目标位置")
        try:
            self.arm_client.set_joint_positions(target, speed_factor=speed_factor)
            time.sleep(0.3)
            logger.info(f"  ✅ 完成")
            return True
        except Exception as e:
            logger.error(f"  ❌ 失败: {e}")
            return False
    
    def move_hand_to_pose(self, pose_name: str, speed_factor: float = 1.0) -> bool:
        """移动灵巧手到指定姿态"""
        if pose_name not in self.hand_poses:
            logger.error(f"❌ 灵巧手姿态不存在: {pose_name}")
            return False
        
        positions = self.hand_poses[pose_name]['positions']
        
        logger.info(f"  ✋ 移动灵巧手到: {pose_name}")
        try:
            self.hand_client.set_joint_positions(
                positions=positions,
                duration=None,
                speed_factor=speed_factor
            )
            time.sleep(0.5)
            logger.info(f"  ✅ 完成")
            return True
        except Exception as e:
            logger.error(f"  ❌ 失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def adjust_single_joint(self, joint_index: int, delta_rad: float, speed_factor: float = 1.0):
        """调整单个关节角度"""
        target = self.arm_client._current_jpos_des.copy()
        target[joint_index] += delta_rad
        
        joint_names = [
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"
        ]
        
        logger.info(f"  🔧 调整 {joint_names[joint_index]}: {delta_rad:+.2f} rad")
        try:
            self.arm_client.set_joint_positions(target, speed_factor=speed_factor)
            time.sleep(0.3)
            logger.info(f"  ✅ 完成")
        except Exception as e:
            logger.error(f"  ❌ 失败: {e}")
    
    def execute_task(self, target_index: int, confirm: bool = True, speak_msg: str = "出现跳闸.") -> None:
        """
        执行完整任务流程
        
        Args:
            target_index: 目标区域索引
            confirm: 是否需要用户确认 (API调用建议设为False)
            speak_msg: 任务完成时的播报内容
            
        Raises:
            TouchSystemError及其子类: 各种可能的错误
        """
        logger.info("\n" + "="*70)
        logger.info(f"🎯 开始执行手机触摸任务 - 目标区域 {target_index}")
        logger.info("="*70)
        
        try:
            # ========== 步骤0: 获取IK解 (自动使用升级版深度获取) ==========
            logger.info(f"\n【步骤0】获取目标区域 {target_index} 的IK解")
            logger.info("-"*70)
            
            # solve_for_target 现在会抛出异常
            ik_result = self.ik_solver.solve_for_target(target_index)
            # result won't be None if no exception raised
            
            self.target_joint_angles, self.target_torso_coord = ik_result
            
            # 🆕 验证Torso范围
            tx, ty, tz = self.target_torso_coord
            
            if self.torso_x_range:
                min_x, max_x = self.torso_x_range
                if not (min_x <= tx <= max_x):
                    msg = f"Torso X坐标超出范围: {tx:.3f} m (允许: {min_x:.3f} ~ {max_x:.3f} m)"
                    logger.error(f"\n❌ {msg}")
                    raise SafetyLimitError(msg)
            
            if self.torso_y_range:
                min_y, max_y = self.torso_y_range
                if not (min_y <= ty <= max_y):
                    msg = f"Torso Y坐标超出范围: {ty:.3f} m (允许: {min_y:.3f} ~ {max_y:.3f} m)"
                    logger.error(f"\n❌ {msg}")
                    raise SafetyLimitError(msg)

            # 用户确认
            if confirm:
                if not self._confirm_execution(target_index):
                    logger.info("\n❌ 任务已取消")
                    return # 取消不视为错误，只是退出
            
            # ========== 正式开始执行 ==========
            with robot_state.safe_arm_control(arm="left", source="phone_touch", timeout=180.0):
                
                # 步骤1-7 保持不变...
                logger.info(f"\n【步骤1】执行预备姿态序列")
                logger.info("-"*70)
                
                prepare_sequence = ["phone_pre_1", "phone_pre_2", "phone_pre_3", "phone_pre_final"]
                for pose in prepare_sequence:
                    if not self.move_arm_to_pose(pose):
                        raise RobotControlError(f"移动到预备姿态失败: {pose}")
                
                logger.info(f"\n【步骤2】设置灵巧手姿态")
                logger.info("-"*70)
                
                if not self.move_hand_to_pose("phone_pre_1"):
                    raise RobotControlError("移动灵巧手失败: phone_pre_1")
                
                logger.info(f"\n【步骤3】移动到目标位置")
                logger.info("-"*70)
                
                if not self.move_arm_to_angles(self.target_joint_angles, speed_factor=1.0):
                    logger.error("❌ [Task] 移动手臂到IK解失败")
                    raise RobotControlError("移动手臂到IK解失败")
                
                time.sleep(1.0)
                
                logger.info(f"\n【步骤4】手腕yaw摆动测试")
                logger.info("-"*70)
                
                WRIST_YAW_INDEX = 6
                
                logger.info(f"  🔄 摆动 {self.wrist_pitch:.2f} rad")
                # self.adjust_single_joint(WRIST_YAW_INDEX, -0.55)
                self.adjust_single_joint(WRIST_YAW_INDEX, self.wrist_pitch) 
                logger.info(f"  🔄 摆动 {-self.wrist_pitch:.2f} rad (归位)")
                # self.adjust_single_joint(WRIST_YAW_INDEX, +0.55)
                self.adjust_single_joint(WRIST_YAW_INDEX, -self.wrist_pitch)
                logger.info(f"\n【步骤5】设置灵巧手恢复原位")
                logger.info("-"*70)
                
                if not self.move_hand_to_pose("close"):
                    logger.error("❌ [Task] 灵巧手复位失败")
                    raise RobotControlError("灵巧手复位失败")

                logger.info(f"\n【步骤6】肘关节收缩")
                logger.info("-"*70)
                
                ELBOW_INDEX = 3
                
                logger.info("  💪 收缩 -0.5 rad")
                self.adjust_single_joint(ELBOW_INDEX, -0.5)

                # 🆕 播报完成信息
                try:
                    TTSClient.speak("报事故，" + speak_msg, wait=False, source="emergency_call")
                except Exception as e:
                    logger.warning(f"⚠️ 语音播报失败: {e}")
                
                logger.info(f"\n【步骤7】反向归位")
                logger.info("-"*70)
                
                return_sequence = ["phone_pre_final", "phone_pre_3", "phone_pre_2", "phone_pre_1"]
                for pose in return_sequence:
                    if not self.move_arm_to_pose(pose):
                        logger.error(f"❌ [Task] 归位失败: {pose}")
                        raise RobotControlError(f"归位失败: {pose}")
                
                logger.info("\n🏁 任务执行完成!")
                
        except KeyboardInterrupt:
            logger.warning("\n\n⚠️  检测到键盘中断，执行安全退出...")
            self.emergency_exit = True
            self._safe_emergency_exit()
            raise # Re-raise after safety exit
        except Exception as e:
            logger.error(f"\n❌ 任务执行失败: {e}")
            import traceback
            traceback.print_exc()
            self.emergency_exit = True
            self._safe_emergency_exit()
            raise # Re-raise exception
    
    def _safe_emergency_exit(self):
        """智能紧急退出 (保持不变)"""
        logger.info("\n" + "="*70)
        logger.info("🚨 执行智能紧急退出")
        logger.info("="*70)
        
        try:
            if self._check_need_lift():
                logger.info("\n🚑 需要先抬起手臂!")
                logger.info("-"*70)
                
                logger.info("💪 收缩肘关节...")
                self.adjust_single_joint(3, -0.5, speed_factor=1.0)
                time.sleep(0.5)
                
                logger.info("🔄 逐步归位...")
                return_sequence = ["phone_pre_final", "phone_pre_3", "phone_pre_2", "phone_pre_1"]
                for pose in return_sequence:
                    self.move_arm_to_pose(pose, speed_factor=1.0)
                    time.sleep(0.5)
                
                logger.info("✅ 安全退出完成")
            else:
                logger.info("\n✅ 手臂安全,直接关闭")
            
        except Exception as e:
            logger.error(f"⚠️  紧急退出过程出错: {e}")
            import traceback
            traceback.print_exc()
    
    def shutdown(self):
        """关闭所有控制器"""
        logger.info("\n🔧 关闭控制器...")
        
        if self.arm_client:
            self.arm_client.stop_control()
            robot_state.reset_arm_state("left")
        
        if self.hand_client:
            self.hand_client.stop_control()
            robot_state.reset_hand_state("left")
        
        logger.info("✅ 已关闭")


def get_mode(val) -> Optional[int]:
    """解析SDK返回的模式值"""
    if isinstance(val, str):
        try:
            val = json.loads(val)
        except Exception:
            pass
    if isinstance(val, dict) and "data" in val:
        return int(val["data"])
    try:
        return int(val)
    except (ValueError, TypeError):
        return None

def main():
    """主程序"""
    # ========== 配置参数 ==========
    TARGET_INDEX = 30
    INTERFACE = "eth0"
    
    # 初始化SDK以获取状态
    ChannelFactoryInitialize(0, INTERFACE)
    sport_client = LocoClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()
    
    # 获取当前模式
    cur_id = get_mode(sport_client.GetFsmId())
    cur_mode = get_mode(sport_client.GetFsmMode())
    
    logger.info("="*70)
    logger.info("📱 手机触摸任务控制器 (升级版深度获取)")
    logger.info("="*70)
    logger.info(f"🔍 检测机器人状态: FSM ID={cur_id}, Mode={cur_mode}")
    
    # 根据 hanger_boot_sequence_run.py 的判断逻辑
    if cur_id == 801 and cur_mode is not None and cur_mode != 2:
        logger.info("✅ 判定为: 走跑运控模式 (Run Mode)")
        # 走跑模式下的参数
        EXPECTED_TORSO_Z = -0.17
        MEASUREMENT_ERROR = [0.005, -0.05, 0.25]
        WRIST_PITCH = -0.70
        TORSO_X_RANGE = (0.25, 0.39)
        TORSO_Y_RANGE = (0.14, 0.38)
    else:
        logger.info("✅ 判定为: 常规运控模式 (Regular Mode)")
        # 常规运控模式下的参数
        EXPECTED_TORSO_Z = -0.15
        MEASUREMENT_ERROR = [-0.01, -0.065, 0.23]
        WRIST_PITCH = -0.60
        TORSO_X_RANGE = (0.23, 0.38)
        TORSO_Y_RANGE = (0.13, 0.38)
    
    TORSO_Z_TOLERANCE = 0.05    # ±5cm容差
    # ==============================
    
    logger.info(f"🎯 目标区域: {TARGET_INDEX}")
    logger.info(f"🌐 网络接口: {INTERFACE}")
    logger.info(f"📏 Torso Z基准: {EXPECTED_TORSO_Z:.3f}m (±{TORSO_Z_TOLERANCE*100:.0f}cm)")
    logger.info(f"📐 测量误差修正: {MEASUREMENT_ERROR}")
    logger.info(f"🤖 手腕下倾角: {WRIST_PITCH} rad")
    logger.info(f"🛡️  X范围: {TORSO_X_RANGE}")
    logger.info(f"🛡️  Y范围: {TORSO_Y_RANGE}")
    logger.info(f"⚠️  安全阈值: X > 0.07m 且 Z > -0.1m")
    logger.info("="*70)
    
    # 🆕 传入动态参数
    controller = PhoneTouchController(
        interface=INTERFACE,
        expected_torso_z=EXPECTED_TORSO_Z,
        torso_z_tolerance=TORSO_Z_TOLERANCE,
        measurement_error=MEASUREMENT_ERROR,
        wrist_pitch=WRIST_PITCH,
        torso_x_range=TORSO_X_RANGE,
        torso_y_range=TORSO_Y_RANGE
    )
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        controller.execute_task(TARGET_INDEX, confirm=True)
        sys.exit(0)
        
    except KeyboardInterrupt:
        logger.warning("\n\n⚠️  收到中断信号")
        sys.exit(1)
    except Exception as e:
        logger.error(f"\n❌ 错误: {e}")
        # import traceback
        # traceback.print_exc()
        sys.exit(1)
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
