"""
G1 机器人手臂控制客户端 - 精简接口版本

提供G1机器人手臂控制的核心功能:
- 17自由度手臂控制 (双臂14DOF + 腰部3DOF, 默认固定腰部)
- 基于官方DDS协议的高层运控接口
- 安全权重过渡控制
- 预定义手势库
"""

import time
import threading
import math
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize


@dataclass
class G1ArmConfig:
    """G1 手臂配置参数"""
    # 控制参数
    default_kp: float = 60.0  # 默认位置增益（油门灵敏度）
    default_kd: float = 1.5   # 默认速度增益（刹车力度）
    default_dq: float = 0.0   # 到达目标位置时的期望速度 (平稳停止)
    default_tau_ff: float = 0.0  # 默认前馈扭矩
    
    # 时间控制参数
    control_dt: float = 0.02  # 控制周期 20ms
    max_joint_velocity: float = 0.5  # 最大关节速度 rad/s
    # max_joint_velocity: float = 1.0  # 最大关节速度 rad/s
    # weight_rate: float = 0.2  # 权重变化率
    weight_rate: float = 0.4  # 权重变化率 ,需要修改initialize_arms中的时间为2.5s 
    
    # 安全参数
    enable_waist_control: bool = False  # 是否启用腰部控制
    
    # 关节限位 (单位: rad) - 基于G1 URDF
    joint_limits: List[Tuple[float, float]] = None
    
    # 🆕 力矩限位 (单位: N·m) - 基于URDF中的effort参数
    torque_limits: List[float] = None
    
    # 🆕 速度限位 (单位: rad/s) - 基于URDF中的velocity参数
    velocity_limits: List[float] = None
    
    def __post_init__(self):
        if self.joint_limits is None:
            # G1手臂关节限位 - 基于URDF文件，按照手臂关节顺序
            self.joint_limits = [
                # 左臂 (7个关节) - 基于URDF中的关节限位
                (-3.0892, 2.6704),   # 左肩俯仰 (pitch): -177°~153°
                (-1.5882, 2.2515),   # 左肩侧摆 (roll): -91°~129°
                (-2.618, 2.618),     # 左肩偏航 (yaw): ±150°
                (-1.0472, 2.0944),   # 左肘屈曲: -60°~120°
                (-1.972222054, 1.972222054),  # 左腕翻滚 (roll): ±113°
                (-1.614429558, 1.614429558),  # 左腕俯仰 (pitch): ±92.5°
                (-1.614429558, 1.614429558),  # 左腕偏航 (yaw): ±92.5°
                
                # 右臂 (7个关节) - 基于URDF中的关节限位
                (-3.0892, 2.6704),   # 右肩俯仰 (pitch): -177°~153°
                (-2.2515, 1.5882),   # 右肩侧摆 (roll): -129°~91° (左右镜像对称)
                (-2.618, 2.618),     # 右肩偏航 (yaw): ±150°
                (-1.0472, 2.0944),   # 右肘屈曲: -60°~120°
                (-1.972222054, 1.972222054),  # 右腕翻滚 (roll): ±113°
                (-1.614429558, 1.614429558),  # 右腕俯仰 (pitch): ±92.5°
                (-1.614429558, 1.614429558),  # 右腕偏航 (yaw): ±92.5°
                
                # 腰部 (3个关节) - 基于URDF中的关节限位
                (-2.618, 2.618),     # 腰部偏航 (yaw): ±150°
                (-0.52, 0.52),       # 腰部侧倾 (roll): ±30°
                (-0.52, 0.52),       # 腰部俯仰 (pitch): ±30°
            ]
        
        if self.torque_limits is None:
            # 🆕 G1手臂力矩限位 - 基于URDF中的effort参数
            self.torque_limits = [
                # 左臂 (7个关节)
                25.0,  # 左肩俯仰: 25 N·m
                25.0,  # 左肩侧摆: 25 N·m
                25.0,  # 左肩偏航: 25 N·m
                25.0,  # 左肘屈曲: 25 N·m
                25.0,  # 左腕翻滚: 25 N·m
                5.0,   # 左腕俯仰: 5 N·m (手腕小关节,力矩较小)
                5.0,   # 左腕偏航: 5 N·m
                
                # 右臂 (7个关节)
                25.0,  # 右肩俯仰: 25 N·m
                25.0,  # 右肩侧摆: 25 N·m
                25.0,  # 右肩偏航: 25 N·m
                25.0,  # 右肘屈曲: 25 N·m
                25.0,  # 右腕翻滚: 25 N·m
                5.0,   # 右腕俯仰: 5 N·m
                5.0,   # 右腕偏航: 5 N·m
                
                # 腰部 (3个关节) - 如果启用
                25.0,  # 腰部偏航: 25 N·m (假设值,URDF未提供)
                25.0,  # 腰部侧倾: 25 N·m
                25.0,  # 腰部俯仰: 25 N·m
            ]
        
        if self.velocity_limits is None:
            # 🆕 G1手臂速度限位 - 基于URDF中的velocity参数
            self.velocity_limits = [
                # 左臂 (7个关节)
                37.0,  # 左肩俯仰: 37 rad/s
                37.0,  # 左肩侧摆: 37 rad/s
                37.0,  # 左肩偏航: 37 rad/s
                37.0,  # 左肘屈曲: 37 rad/s
                37.0,  # 左腕翻滚: 37 rad/s
                22.0,  # 左腕俯仰: 22 rad/s (手腕小关节,速度较低)
                22.0,  # 左腕偏航: 22 rad/s
                
                # 右臂 (7个关节)
                37.0,  # 右肩俯仰: 37 rad/s
                37.0,  # 右肩侧摆: 37 rad/s
                37.0,  # 右肩偏航: 37 rad/s
                37.0,  # 右肘屈曲: 37 rad/s
                37.0,  # 右腕翻滚: 37 rad/s
                22.0,  # 右腕俯仰: 22 rad/s
                22.0,  # 右腕偏航: 22 rad/s
                
                # 腰部 (3个关节) - 如果启用
                37.0,  # 腰部偏航: 37 rad/s (假设值,URDF未提供)
                37.0,  # 腰部侧倾: 37 rad/s
                37.0,  # 腰部俯仰: 37 rad/s
            ]


class JointIndex:
    """G1机器人关节索引枚举 (共35个电机)"""
    # 左腿 (6个关节: 索引 0-5)
    kLeftHipPitch, kLeftHipRoll, kLeftHipYaw = 0, 1, 2      # 左髋俯仰/侧摆/偏航
    kLeftKnee, kLeftAnkle, kLeftAnkleRoll = 3, 4, 5         # 左膝/左踝俯仰/左踝侧摆
    
    # 右腿 (6个关节: 索引 6-11)
    kRightHipPitch, kRightHipRoll, kRightHipYaw = 6, 7, 8   # 右髋俯仰/侧摆/偏航
    kRightKnee, kRightAnkle, kRightAnkleRoll = 9, 10, 11    # 右膝/右踝俯仰/右踝侧摆
    
    # 腰部 (3个关节: 索引 12-14)
    kWaistYaw, kWaistRoll, kWaistPitch = 12, 13, 14         # 腰部偏航/侧倾/俯仰
    
    # 左臂 (7个关节: 索引 15-21)
    kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw = 15, 16, 17  # 左肩俯仰/侧摆/偏航
    kLeftElbow, kLeftWristRoll, kLeftWristPitch, kLeftWristYaw = 18, 19, 20, 21  # 左肘/左腕翻滚/俯仰/偏航
    
    # 右臂 (7个关节: 索引 22-28)
    kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw = 22, 23, 24  # 右肩俯仰/侧摆/偏航
    kRightElbow, kRightWristRoll, kRightWristPitch, kRightWristYaw = 25, 26, 27, 28  # 右肘/右腕翻滚/俯仰/偏航
    
    # 预留关节 (索引 29: 用于权重控制)
    kNotUsedJoint = 29


class G1ArmClient:
    """
    G1 机器人手臂控制客户端
    
    参数:
        interface: 网络接口名称 (默认 "eth0")
        config: 配置参数
    
    示例:
        arm = G1ArmClient(interface="eth0")
        arm.initialize_arms()  # 初始化到自然位
        arm.set_joint_positions([0.0]*14, duration=3.0)  # 设置关节位置
        arm.stop_control()  # 安全停止
    """
    
    def __init__(self, interface: str = "eth0", config: Optional[G1ArmConfig] = None):
        self.config = config or G1ArmConfig()
        self._interface = interface
        
        # DDS通信设置
        self._cmd_topic = "rt/arm_sdk"      # 命令发布话题
        self._state_topic = "rt/lowstate"   # 状态订阅话题
        self._cmd_publisher: Optional[ChannelPublisher] = None
        self._state_subscriber: Optional[ChannelSubscriber] = None
        
        # 状态缓存
        self._latest_state: Optional[Any] = None
        self._state_lock = threading.Lock()
        
        # 手臂关节索引 - 与C++版本完全一致
        self._arm_joints = [
            # 左臂 (7个关节)
            JointIndex.kLeftShoulderPitch, JointIndex.kLeftShoulderRoll,
            JointIndex.kLeftShoulderYaw, JointIndex.kLeftElbow,
            JointIndex.kLeftWristRoll, JointIndex.kLeftWristPitch, JointIndex.kLeftWristYaw,
            # 右臂 (7个关节)
            JointIndex.kRightShoulderPitch, JointIndex.kRightShoulderRoll,
            JointIndex.kRightShoulderYaw, JointIndex.kRightElbow,
            JointIndex.kRightWristRoll, JointIndex.kRightWristPitch, JointIndex.kRightWristYaw,
        ]
        
        # 如果启用腰部控制，添加腰部关节 (3个关节)
        if self.config.enable_waist_control:
            self._arm_joints.extend([
                JointIndex.kWaistYaw, JointIndex.kWaistRoll, JointIndex.kWaistPitch
            ])
        
        self.ARM_JOINT_COUNT = len(self._arm_joints)
        
        # 控制参数
        self._weight = 0.0  # 当前控制权重 (0~1)
        self._delta_weight = self.config.weight_rate * self.config.control_dt  # 每步权重变化量
        self._max_joint_delta = self.config.max_joint_velocity * self.config.control_dt  # 每步最大关节变化量
        self._sleep_duration = self.config.control_dt  # 睡眠时长
        
        # 预定义位置 - 自然下垂位置 (14自由度)
        self._nature_pos = [
            # 左臂 (7个关节)
            0.243, 0.173, -0.016, 0.796, 0.090, 0.027, -0.008,
            # 右臂 (7个关节)
            0.250, -0.175, 0.025, 0.801, -0.111, 0.035, 0.009
        ]
        
        # 当前期望位置状态 - 用于跟踪运动状态，避免位置跳变
        self._current_jpos_des = [0.0] * self.ARM_JOINT_COUNT
        
        # 初始化DDS连接
        self._init_dds_connection()
    
    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """
        限制数值在指定范围内
        
        参数:
            value: 输入值
            min_val: 最小值
            max_val: 最大值
        
        返回:
            限制后的值
        """
        return max(min_val, min(max_val, value))
    
    def _init_dds_connection(self):
        """初始化DDS通信连接"""
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        
        if self._interface:
            ChannelFactoryInitialize(0, self._interface)
        
        # 初始化命令发布者
        self._cmd_publisher = ChannelPublisher(self._cmd_topic, LowCmd_)
        self._cmd_publisher.Init()
        
        # 初始化状态订阅者
        self._state_subscriber = ChannelSubscriber(self._state_topic, LowState_)
        self._state_subscriber.Init(self._state_callback, 10)
        
        time.sleep(1.0)  # 等待连接稳定
    
    def _state_callback(self, msg):
        """状态消息回调函数 - 线程安全更新"""
        with self._state_lock:
            self._latest_state = msg
    
    def _publish_command(self, cmd) -> bool:
        """
        发布控制命令到DDS
        
        参数:
            cmd: LowCmd_ 消息对象
        
        返回:
            bool: 是否成功
        """
        try:
            self._cmd_publisher.Write(cmd)
            return True
        except Exception as e:
            print(f"[G1Arm] 命令发布失败: {e}")
            return False
    
    def _create_arm_command(
        self, positions: List[float], velocities: Optional[List[float]] = None,
        torques: Optional[List[float]] = None, kp: Optional[float] = None,
        kd: Optional[float] = None, weight: Optional[float] = None
    ):
        """
        创建手臂控制命令 - 基于官方LowCmd_结构
        
        参数:
            positions: 关节目标位置 (rad)
            velocities: 关节目标速度 (rad/s)
            torques: 前馈扭矩 (N·m)
            kp: 位置增益
            kd: 速度增益
            weight: 控制权重 (0~1)
        
        返回:
            LowCmd_ 消息对象
        """
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, MotorCmd_
        
        # 参数默认值处理
        velocities = velocities or [self.config.default_dq] * self.ARM_JOINT_COUNT
        torques = torques or [self.config.default_tau_ff] * self.ARM_JOINT_COUNT
        kp = kp if kp is not None else self.config.default_kp
        kd = kd if kd is not None else self.config.default_kd
        weight = weight if weight is not None else self._weight
        
        # 🆕 安全限位检查
        velocities = self._clamp_velocities(velocities)
        torques = self._clamp_torques(torques)
        
        # 创建35个电机命令数组 (G1机器人总电机数)
        motor_cmds = [MotorCmd_(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=0) for _ in range(35)]
        
        # 设置权重 - 使用 kNotUsedJoint(29) 作为权重控制通道
        motor_cmds[JointIndex.kNotUsedJoint].mode = 1
        motor_cmds[JointIndex.kNotUsedJoint].q = float(weight)
        
        # 🆕 固定腰部偏航关节到0位
        motor_cmds[JointIndex.kWaistYaw].mode = 1  # 使能模式
        motor_cmds[JointIndex.kWaistYaw].q = 0.0   # 目标位置: 0 rad
        motor_cmds[JointIndex.kWaistYaw].dq = 0.0  # 目标速度: 0 rad/s
        motor_cmds[JointIndex.kWaistYaw].tau = 0.0 # 前馈扭矩: 0 N·m
        motor_cmds[JointIndex.kWaistYaw].kp = 60.0 # 位置增益 (较高的刚度保持固定)
        motor_cmds[JointIndex.kWaistYaw].kd = 1.5  # 速度增益
        
        # 设置手臂关节命令
        for i, joint_idx in enumerate(self._arm_joints):
            motor_cmds[joint_idx].mode = 1  # 使能模式
            motor_cmds[joint_idx].q = float(positions[i])      # 目标位置
            motor_cmds[joint_idx].dq = float(velocities[i])    # 目标速度
            motor_cmds[joint_idx].tau = float(torques[i])      # 前馈扭矩
            motor_cmds[joint_idx].kp = float(kp)               # 位置增益
            motor_cmds[joint_idx].kd = float(kd)               # 速度增益
        
        return LowCmd_(mode_pr=0, mode_machine=0, motor_cmd=motor_cmds, reserve=[0]*4, crc=0)
    
    # 🆕 添加安全限位方法
    def _clamp_velocities(self, velocities: List[float]) -> List[float]:
        """
        限制速度在安全范围内
        
        参数:
            velocities: 输入速度列表
        
        返回:
            限制后的速度列表
        """
        limits = self.config.velocity_limits[:self.ARM_JOINT_COUNT]
        return [
            self._clamp(vel, -limit, limit)
            for vel, limit in zip(velocities, limits)
        ]
    
    def _clamp_torques(self, torques: List[float]) -> List[float]:
        """
        限制力矩在安全范围内
        
        参数:
            torques: 输入力矩列表
        
        返回:
            限制后的力矩列表
        """
        limits = self.config.torque_limits[:self.ARM_JOINT_COUNT]
        return [
            self._clamp(tau, -limit, limit)
            for tau, limit in zip(torques, limits)
        ]
    
    def get_safety_limits(self) -> Dict[str, Any]:
        """
        🆕 获取所有安全限位信息
        
        返回:
            包含位置、速度、力矩限位的字典
        """
        return {
            'position_limits': self.config.joint_limits[:self.ARM_JOINT_COUNT],
            'velocity_limits': self.config.velocity_limits[:self.ARM_JOINT_COUNT],
            'torque_limits': self.config.torque_limits[:self.ARM_JOINT_COUNT],
            'joint_count': self.ARM_JOINT_COUNT,
            'waist_enabled': self.config.enable_waist_control
        }
    
    def smooth_transition(
        self, start_positions: Optional[List[float]], target_positions: List[float],
        duration: float, description: str = ""
    ) -> bool:
        """
        平滑过渡到目标位置
        
        使用与C++例程相同的控制算法:
        1. 通过 _current_jpos_des 跟踪当前期望位置
        2. 每步限制最大变化量 (防止速度过快)
        3. 逐步趋向目标位置
        
        参数:
            start_positions: 起始位置 (None表示使用当前_current_jpos_des)
            target_positions: 目标位置
            duration: 过渡时长(秒)
            description: 描述信息 (用于日志输出)
        
        返回:
            bool: 是否成功
        """
        if description:
            print(f"[G1Arm] {description}...")
        
        time_steps = int(duration / self.config.control_dt)
        
        # 只有明确提供 start_positions 时才更新
        if start_positions is not None:
            self._current_jpos_des = start_positions.copy()
        # 否则保持使用当前的 _current_jpos_des
        
        start_time = time.time()
        for i in range(time_steps):
            # 更新期望位置 - 限制每步的最大变化量
            for j in range(len(self._current_jpos_des)):
                delta = target_positions[j] - self._current_jpos_des[j]
                delta = self._clamp(delta, -self._max_joint_delta, self._max_joint_delta)
                self._current_jpos_des[j] += delta
            
            # 创建并发布命令
            cmd = self._create_arm_command(self._current_jpos_des)
            if not self._publish_command(cmd):
                return False
            
            # 精确延时 - 防止累积误差
            expected_time = start_time + (i + 1) * self._sleep_duration
            sleep_time = expected_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if description:
            print(f"[G1Arm] {description}完成")
        return True
    
    def initialize_arms(self) -> bool:
        """
        初始化手臂到自然位置
        
        使用权重过渡算法:
        1. 逐步增加权重从 0 到 1 (5秒过渡)
        2. 同时进行位置插值到自然位置
        3. 权重使用 weight² 的平方关系 (平滑加速)
        
        返回:
            bool: 是否成功
        """
        # 获取当前关节位置
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False
        
        self._current_jpos_des = current_positions.copy()
        
        # init_time = 5.0  # 初始化时间 5秒
        init_time = 2.5  # 初始化时间 5秒
        time_steps = int(init_time / self.config.control_dt)
        self._weight = 0.0
        
        start_time = time.time()
        for i in range(time_steps):
            # 逐步增加权重
            self._weight += self._delta_weight
            self._weight = self._clamp(self._weight, 0.0, 1.0)
            
            # 计算过渡位置 - 线性插值
            phase = 1.0 if i == time_steps - 1 else float(i) / time_steps
            transition_positions = [
                self._nature_pos[j] * phase + current_positions[j] * (1 - phase)
                for j in range(self.ARM_JOINT_COUNT)
            ]
            
            self._current_jpos_des = transition_positions.copy()
            
            # 创建并发布命令 (权重使用平方关系)
            cmd = self._create_arm_command(transition_positions, weight=self._weight * self._weight)
            if not self._publish_command(cmd):
                return False
            
            # 精确延时
            expected_time = start_time + (i + 1) * self._sleep_duration
            sleep_time = expected_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 确保权重达到 1.0
        while self._weight < 1.0:
            self._weight += self._delta_weight
            self._weight = self._clamp(self._weight, 0.0, 1.0)
            cmd = self._create_arm_command(self._nature_pos, weight=self._weight * self._weight)
            if not self._publish_command(cmd):
                return False
            self._current_jpos_des = self._nature_pos.copy()
            time.sleep(self._sleep_duration)
        
        return True
    
    def stop_control(self) -> bool:
        """
        停止控制并恢复到自然位置
        
        🆕 使用 set_joint_positions 实现,自动计算时间
        
        返回:
            bool: 是否成功
        """
        print("[G1Arm] 停止控制...")
        
        # 🎯 直接使用 set_joint_positions,自动计算返回时间
        # 优势:
        # 1. 自动计算最优时间 (基于实际位置差)
        # 2. 复用现有逻辑,代码更简洁
        # 3. 保持一致的控制行为
        success = self.set_joint_positions(
            self._nature_pos,
            duration=None,  # 自动计算
            speed_factor=1.0
        )
        
        if not success:
            print("[G1Arm] 返回自然位失败")
            return False
        
        # 逐步降低权重到 0
        while self._weight > 0.0:
            self._weight -= self._delta_weight
            self._weight = self._clamp(self._weight, 0.0, 1.0)
            cmd = self._create_arm_command(
                self._nature_pos, 
                weight=self._weight * self._weight
            )
            if not self._publish_command(cmd):
                return False
            self._current_jpos_des = self._nature_pos.copy()
            time.sleep(self._sleep_duration)
        
        print("[G1Arm] 控制已停止")
        return True
    
    def set_joint_positions(
        self, 
        positions: List[float], 
        duration: Optional[float] = None,
        speed_factor: float = 1.0,
        kp: Optional[float] = None, 
        kd: Optional[float] = None
    ) -> bool:
        """
        设置关节位置 - 智能时间控制
        
        参数:
            positions: 关节位置列表（弧度）
            duration: 执行时间(秒) - None时自动计算
            speed_factor: 速度因子 (>1加快, <1减慢)
            kp: 位置增益 (可选)
            kd: 速度增益 (可选)
        
        返回:
            bool: 是否成功
        
        示例:
            # 自动计算时间
            arm.set_joint_positions(pose)
            
            # 指定时间
            arm.set_joint_positions(pose, duration=3.0)
            
            # 2倍速执行
            arm.set_joint_positions(pose, speed_factor=2.0)
        """
        if len(positions) != self.ARM_JOINT_COUNT:
            print(f"[G1Arm] 错误: 位置数量({len(positions)})与关节数({self.ARM_JOINT_COUNT})不匹配")
            return False
        
        # 关节限位检查
        limits = self.config.joint_limits[:self.ARM_JOINT_COUNT]
        clamped_positions = [
            max(min_val, min(max_val, pos))
            for pos, (min_val, max_val) in zip(positions, limits)
        ]
        
        # 🎯 智能计算时间（自包含，无需额外函数）
        if duration is None:
            max_delta = max(
                abs(clamped_positions[i] - self._current_jpos_des[i])
                for i in range(len(self._current_jpos_des))
            )
            
            required_steps = math.ceil(max_delta / self._max_joint_delta)
            base_duration = required_steps * self.config.control_dt
            duration = base_duration * 1.1 / speed_factor  # 20%余量 + 速度因子
            duration = max(duration, 0.5)  # 最小0.5秒
            
            print(f"[G1Arm] 自动时间: {duration:.2f}s "
                  f"(Δ={max_delta:.3f}rad, 速度={speed_factor}x)")
        
        return self.smooth_transition(None, clamped_positions, duration, "")

    def set_arm_pose(self, pose_name: str) -> bool:
        """
        设置手臂到预定义姿态
        
        参数:
            pose_name: 姿态名称 (nature, open_arms, hello_1等)
        
        返回:
            bool: 是否成功
        """
        pose = G1ArmGestures.get_pose(pose_name, self.config.enable_waist_control)
        if pose is None:
            return False
        return self.set_joint_positions(pose)
    
    def get_current_joint_positions(self, timeout: float = 2.0) -> Optional[List[float]]:
        """
        获取当前手臂关节位置
        
        参数:
            timeout: 超时时间(秒)
        
        返回:
            关节位置列表 (rad) 或 None (超时)
        """
        start_time = time.time()
        time.sleep(0.1)  # 等待首次消息
        
        while time.time() - start_time < timeout:
            with self._state_lock:
                if self._latest_state is not None:
                    state = self._latest_state
                    # 检查消息是否包含足够的电机状态
                    if hasattr(state, 'motor_state') and len(state.motor_state) >= 35:
                        return [float(state.motor_state[idx].q) for idx in self._arm_joints]
            time.sleep(0.01)
        
        return None
    
    def get_joint_states(self, timeout: float = 2.0) -> Optional[Dict[str, Any]]:
        """
        获取详细的关节状态
        
        参数:
            timeout: 超时时间(秒)
        
        返回:
            包含位置、速度、扭矩、温度等信息的字典, 或 None (超时)
            字典格式:
            {
                'positions': [q1, q2, ...],      # 关节位置 (rad)
                'velocities': [dq1, dq2, ...],   # 关节速度 (rad/s)
                'torques': [tau1, tau2, ...],    # 关节扭矩 (N·m)
                'temperatures': [t1, t2, ...]    # 电机温度 (℃)
            }
        """
        start_time = time.time()
        time.sleep(0.1)
        
        while time.time() - start_time < timeout:
            with self._state_lock:
                if self._latest_state is not None:
                    state = self._latest_state
                    if hasattr(state, 'motor_state') and len(state.motor_state) >= 35:
                        joint_states = {
                            'positions': [],      # 位置
                            'velocities': [],     # 速度
                            'torques': [],        # 扭矩
                            'temperatures': []    # 温度
                        }
                        for idx in self._arm_joints:
                            ms = state.motor_state[idx]
                            joint_states['positions'].append(float(ms.q))
                            joint_states['velocities'].append(float(ms.dq))
                            joint_states['torques'].append(float(ms.tau_est))
                            joint_states['temperatures'].append(
                                ms.temperature[0] if hasattr(ms, 'temperature') else 0
                            )
                        return joint_states
            time.sleep(0.01)
        
        return None


class G1ArmGestures:
    """预定义手臂姿态库 - 基于URDF关节限位优化"""
    
    @staticmethod
    def get_pose(pose_name: str, include_waist: bool = False) -> Optional[List[float]]:
        """
        获取预定义姿态
        
        参数:
            pose_name: 姿态名称
            include_waist: 是否包含腰部关节
        
        返回:
            关节角度列表 (rad) 或 None (姿态不存在)
        """
        kPi_2 = math.pi / 2
        
        # 基础手臂姿态 (14自由度) - 基于URDF限位优化
        arm_poses = {
            "rest": [0.0] * 14,  # 零位姿态
            
            "nature": [
                # 左臂 (7个关节) - 自然下垂位
                0.243,   # 肩俯仰: 13.9°
                0.173,   # 肩侧摆: 9.9°
                -0.016,  # 肩偏航: -0.9°
                0.796,   # 肘屈曲: 45.6°
                0.090,   # 腕翻滚: 5.2°
                0.027,   # 腕俯仰: 1.5°
                -0.008,  # 腕偏航: -0.5°
                # 右臂 (7个关节) - 自然下垂位
                0.250,   # 肩俯仰: 14.3°
                -0.175,  # 肩侧摆: -10.0°
                0.025,   # 肩偏航: 1.4°
                0.801,   # 肘屈曲: 45.9°
                -0.111,  # 腕翻滚: -6.4°
                0.035,   # 腕俯仰: 2.0°
                0.009    # 腕偏航: 0.5°
            ],
            
            "open_arms": [
                # 左臂 - 水平张开 (90°外展)
                0.0,     # 肩俯仰: 0°
                kPi_2,   # 肩侧摆: 90° (向外展开)
                0.0,     # 肩偏航: 0°
                kPi_2,   # 肘屈曲: 90°
                0.0,     # 腕翻滚: 0°
                0.0,     # 腕俯仰: 0°
                0.0,     # 腕偏航: 0°
                # 右臂 - 水平张开 (考虑 roll 关节限位的镜像)
                0.0,     # 肩俯仰: 0°
                -kPi_2,  # 肩侧摆: -90° (向外展开, 镜像对称)
                0.0,     # 肩偏航: 0°
                kPi_2,   # 肘屈曲: 90°
                0.0,     # 腕翻滚: 0°
                0.0,     # 腕俯仰: 0°
                0.0,     # 腕偏航: 0°
            ]
        }
        
        # 检查姿态是否存在
        if pose_name not in arm_poses:
            return None
        
        pose = arm_poses[pose_name].copy()
        
        # 如果包含腰部，添加腰部零位 (3个关节)
        if include_waist:
            pose.extend([0.0, 0.0, 0.0])  # 腰部偏航、侧倾、俯仰
        
        return pose
