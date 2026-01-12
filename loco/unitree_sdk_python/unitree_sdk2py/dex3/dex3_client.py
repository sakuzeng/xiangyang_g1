"""
Dex3 灵巧手控制客户端 - 精简接口版本
实际上smooth_transition每个控制周期控制角度均为0.1rad,是写死的，根本不能根据duration来控制速度,之后再优化
提供宇树 Dex3-1 力控灵巧手控制的核心功能:
- 7自由度关节控制（3指 + 拇指旋转）
- 触觉传感器数据读取（9个传感器，每个3x4点阵）
- 左右手支持
- 预定义手势库
重要函数： set_joint_positions，自动计算执行时间（）
- control_dt: 控制周期，默认20ms
- max_joint_velocity: 最大关节速度，默认0.5 rad/s 想要加速控制响应，可以调整此参数,最大3 rad/s。
- 最大关节增量 = max_joint_velocity * control_dt 0.5*0.02=0.01 rad 每次控制周期
- set_joint_positions计算出时间后调用smooth_transition，每次控制周期内，关节位置变化不超过最大增量e
- 智能时间计算：根据当前位置与目标位置的最大差值，计算所需时间，添加时间余量
- 安全限位检查：新增力矩和速度限位，确保控制命令在安全范围内
- 初始化过程增强：读取当前关节位置，检查异常数据并使用安全值替代，确保初始化过程稳定可靠
- 停止控制优化：使用 set_joint_positions 实现返回自然位置，自动计算时间，简化代码结构
- 扭矩设为0，防止意外
"""
import time
import threading
import contextlib
import math
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize


@dataclass
class Dex3Config:
    """Dex3 灵巧手配置参数"""
    # 关节限位 (单位: rad) - 基于官方URDF
    joint_limits_left: List[Tuple[float, float]] = None
    joint_limits_right: List[Tuple[float, float]] = None
    
    # 🆕 力矩限位 (单位: N·m) - 基于URDF中的effort参数
    torque_limits_left: List[float] = None
    torque_limits_right: List[float] = None
    
    # 🆕 速度限位 (单位: rad/s) - 基于URDF中的velocity参数
    velocity_limits_left: List[float] = None
    velocity_limits_right: List[float] = None
    
    # 控制增益默认值
    default_kp: float = 1.5
    default_kd: float = 0.1
    default_dq: float = 0.0
    default_tau_ff: float = 0.0
    
    # 时间控制参数
    control_dt: float = 0.02  # 控制周期 20ms
    max_joint_velocity: float = 1.0  # 最大关节速度 rad/s
    
    # 安全参数 (已废弃,使用 torque_limits)
    max_torque: float = 2.0  # 最大扭矩 (N·m) - 向后兼容
    
    def __post_init__(self):
        if self.joint_limits_left is None:
            # 左手关节限位 - 基于URDF精确值
            self.joint_limits_left = [
                (-1.0472, 1.0472),   # 左拇指外展/内收 (Y轴旋转, ±60°)
                (-0.6109, 1.0472),   # 左拇指第一指节屈曲 (Z轴旋转, -35°~60°)
                (0.0, 1.7453),       # 左拇指第二指节屈曲 (Z轴旋转, 0°~100°)
                (-1.5708, 0.0),      # 左中指基部屈曲 (Z轴旋转, -90°~0°)
                (-1.7453, 0.0),      # 左中指指尖屈曲 (Z轴旋转, -100°~0°)
                (-1.5708, 0.0),      # 左食指基部屈曲 (Z轴旋转, -90°~0°)
                (-1.7453, 0.0),      # 左食指指尖屈曲 (Z轴旋转, -100°~0°)
            ]
        
        if self.joint_limits_right is None:
            # 右手关节限位 - 基于URDF精确值
            self.joint_limits_right = [
                (-1.0472, 1.0472),   # 右拇指外展/内收 (Y轴旋转, ±60°)
                (-1.0472, 0.6109),   # 右拇指第一指节屈曲 (Z轴旋转, -60°~35°)
                (-1.7453, 0.0),      # 右拇指第二指节屈曲 (Z轴旋转, -100°~0°)
                (0.0, 1.5708),       # 右中指基部屈曲 (Z轴旋转, 0°~90°)
                (0.0, 1.7453),       # 右中指指尖屈曲 (Z轴旋转, 0°~100°)
                (0.0, 1.5708),       # 右食指基部屈曲 (Z轴旋转, 0°~90°)
                (0.0, 1.7453),       # 右食指指尖屈曲 (Z轴旋转, 0°~100°)
            ]
        
        if self.torque_limits_left is None:
            # 🆕 左手力矩限位 - 基于URDF中的effort参数
            self.torque_limits_left = [
                2.45,  # 左拇指外展/内收 (thumb_0): 2.45 N·m
                1.4,   # 左拇指第一指节 (thumb_1): 1.4 N·m
                1.4,   # 左拇指第二指节 (thumb_2): 1.4 N·m
                1.4,   # 左中指基部 (middle_0): 1.4 N·m
                1.4,   # 左中指指尖 (middle_1): 1.4 N·m
                1.4,   # 左食指基部 (index_0): 1.4 N·m
                1.4,   # 左食指指尖 (index_1): 1.4 N·m
            ]
        
        if self.torque_limits_right is None:
            # 🆕 右手力矩限位 - 基于URDF中的effort参数 (与左手相同)
            self.torque_limits_right = [
                2.45,  # 右拇指外展/内收 (thumb_0): 2.45 N·m
                1.4,   # 右拇指第一指节 (thumb_1): 1.4 N·m
                1.4,   # 右拇指第二指节 (thumb_2): 1.4 N·m
                1.4,   # 右中指基部 (middle_0): 1.4 N·m
                1.4,   # 右中指指尖 (middle_1): 1.4 N·m
                1.4,   # 右食指基部 (index_0): 1.4 N·m
                1.4,   # 右食指指尖 (index_1): 1.4 N·m
            ]
        
        if self.velocity_limits_left is None:
            # 🆕 左手速度限位 - 基于URDF中的velocity参数
            self.velocity_limits_left = [
                3.14,  # 左拇指外展/内收 (thumb_0): 3.14 rad/s
                12.0,  # 左拇指第一指节 (thumb_1): 12 rad/s
                12.0,  # 左拇指第二指节 (thumb_2): 12 rad/s
                12.0,  # 左中指基部 (middle_0): 12 rad/s
                12.0,  # 左中指指尖 (middle_1): 12 rad/s
                12.0,  # 左食指基部 (index_0): 12 rad/s
                12.0,  # 左食指指尖 (index_1): 12 rad/s
            ]
        
        if self.velocity_limits_right is None:
            # 🆕 右手速度限位 - 基于URDF中的velocity参数 (与左手相同)
            self.velocity_limits_right = [
                3.14,  # 右拇指外展/内收 (thumb_0): 3.14 rad/s
                12.0,  # 右拇指第一指节 (thumb_1): 12 rad/s
                12.0,  # 右拇指第二指节 (thumb_2): 12 rad/s
                12.0,  # 右中指基部 (middle_0): 12 rad/s
                12.0,  # 右中指指尖 (middle_1): 12 rad/s
                12.0,  # 右食指基部 (index_0): 12 rad/s
                12.0,  # 右食指指尖 (index_1): 12 rad/s
            ]


class Dex3Client:
    """
    Dex3 灵巧手控制客户端
    
    Args:
        hand: 手的类型 ("left" 或 "right")
        interface: 网络接口名称 (默认 "eth0")
        config: 配置参数
    
    Example:
        dex3 = Dex3Client(hand="right", interface="eth0")
        dex3.initialize_hand()
        dex3.set_gesture("open")
        dex3.stop_control()
    """
    
    def __init__(
        self, 
        hand: str = "right", 
        interface: str = "eth0",
        config: Optional[Dex3Config] = None
    ):
        if hand not in ["left", "right"]:
            raise ValueError("hand 必须是 'left' 或 'right'")
        
        self.hand = hand
        self.config = config or Dex3Config()
        self._interface = interface
        
        # DDS通信设置
        self._cmd_topic = f"rt/dex3/{hand}/cmd"
        self._state_topic = f"rt/dex3/{hand}/state"
        self._cmd_publisher: Optional[ChannelPublisher] = None
        self._state_subscriber: Optional[ChannelSubscriber] = None
        
        # 状态缓存
        self._latest_state: Optional[Any] = None
        self._state_lock = threading.Lock()
        
        # 常量
        self.MOTOR_MAX = 7
        self.SENSOR_MAX = 9
        
        # 控制参数
        self._max_joint_delta = self.config.max_joint_velocity * self.config.control_dt
        self._sleep_duration = self.config.control_dt
        self._current_jpos_des = [0.0] * self.MOTOR_MAX
        
        # 预定义位置 - 基于实际弧度值
        self._nature_pos = (
            [-0.029, -1.019, -1.667, 1.551, 1.702, 1.568, 1.710] if hand == "right"
            else [-0.028, 1.010, 1.511, -1.582, -1.779, -1.647, -1.827]
        )
        self._open_pos = (
            [-0.029, 0.587, 0.052, -0.053, -0.034, -0.022, -0.016] if hand == "right"
            else [0.005, -0.616, -0.085, -0.019, -0.035, -0.018, -0.025]
        )
        
        # 初始化DDS连接
        self._init_dds_connection()
    
    def _init_dds_connection(self):
        """初始化DDS连接"""
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
        
        if self._interface:
            ChannelFactoryInitialize(0, self._interface)
        
        self._cmd_publisher = ChannelPublisher(self._cmd_topic, HandCmd_)
        self._cmd_publisher.Init()
        
        self._state_subscriber = ChannelSubscriber(self._state_topic, HandState_)
        self._state_subscriber.Init(self._state_callback, 10)
        
        time.sleep(1.0)
    
    def _state_callback(self, msg):
        """状态消息回调"""
        with self._state_lock:
            self._latest_state = msg
    
    def read_state(self, timeout: float = 1.0) -> Optional[Any]:
        """
        读取灵巧手状态
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            HandState_ 消息或 None
        """
        start_time = time.time()
        time.sleep(0.1)
        
        while time.time() - start_time < timeout:
            with self._state_lock:
                if self._latest_state is not None:
                    return self._latest_state
            time.sleep(0.01)
        
        return None
    
    def _get_joint_limits(self) -> List[Tuple[float, float]]:
        """获取当前手的关节限位"""
        return (
            self.config.joint_limits_left if self.hand == "left"
            else self.config.joint_limits_right
        )
    
    # 🆕 添加获取力矩和速度限位的方法
    def _get_torque_limits(self) -> List[float]:
        """获取当前手的力矩限位"""
        return (
            self.config.torque_limits_left if self.hand == "left"
            else self.config.torque_limits_right
        )
    
    def _get_velocity_limits(self) -> List[float]:
        """获取当前手的速度限位"""
        return (
            self.config.velocity_limits_left if self.hand == "left"
            else self.config.velocity_limits_right
        )
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """限制值在指定范围内"""
        return max(min_val, min(max_val, value))
    
    # 🆕 添加安全限位方法
    def _clamp_velocities(self, velocities: List[float]) -> List[float]:
        """
        限制速度在安全范围内
        
        参数:
            velocities: 输入速度列表
        
        返回:
            限制后的速度列表
        """
        limits = self._get_velocity_limits()
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
        limits = self._get_torque_limits()
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
            'position_limits': self._get_joint_limits(),
            'velocity_limits': self._get_velocity_limits(),
            'torque_limits': self._get_torque_limits(),
            'joint_count': self.MOTOR_MAX,
            'hand_type': self.hand
        }
    
    def _create_hand_command(
        self,
        positions: List[float],
        velocities: Optional[List[float]] = None,
        torques: Optional[List[float]] = None,
        kp: Optional[float] = None,
        kd: Optional[float] = None
    ):
        """创建手部控制命令 - 🆕 添加安全限位"""
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
        
        if len(positions) != self.MOTOR_MAX:
            raise ValueError(f"位置数量({len(positions)})与关节数({self.MOTOR_MAX})不匹配")
        
        velocities = velocities or [self.config.default_dq] * self.MOTOR_MAX
        torques = torques or [self.config.default_tau_ff] * self.MOTOR_MAX
        kp = kp if kp is not None else self.config.default_kp
        kd = kd if kd is not None else self.config.default_kd
        
        # 🆕 安全限位检查
        velocities = self._clamp_velocities(velocities)
        torques = self._clamp_torques(torques)
        
        motor_cmds = [
            MotorCmd_(
                mode=1,
                q=float(positions[i]),
                dq=float(velocities[i]),
                tau=float(torques[i]),
                kp=float(kp),
                kd=float(kd),
                reserve=0
            )
            for i in range(self.MOTOR_MAX)
        ]
        
        return HandCmd_(motor_cmd=motor_cmds, reserve=[0, 0, 0, 0])
    
    def _publish_command(self, cmd) -> bool:
        """发布命令消息"""
        if self._cmd_publisher is None or cmd is None:
            return False
        
        try:
            self._cmd_publisher.Write(cmd)
            return True
        except Exception as e:
            print(f"[Dex3] 发布命令失败: {e}")
            return False
    
    def smooth_transition(
        self,
        start_positions: Optional[List[float]],
        target_positions: List[float],
        duration: float,
        description: str = ""
    ) -> bool:
        """
        平滑过渡到目标位置
        
        Args:
            start_positions: 起始位置 (None表示使用当前_current_jpos_des)
            target_positions: 目标位置
            duration: 过渡时长(秒)
            description: 描述信息
        """
        if description:
            print(f"[Dex3] {description}...")
        
        time_steps = int(duration / self.config.control_dt)
        
        if start_positions is not None:
            self._current_jpos_des = start_positions.copy()
        
        start_time = time.time()
        for i in range(time_steps):
            for j in range(len(self._current_jpos_des)):
                delta = target_positions[j] - self._current_jpos_des[j]
                delta = self._clamp(delta, -self._max_joint_delta, self._max_joint_delta)
                self._current_jpos_des[j] += delta
            
            cmd = self._create_hand_command(self._current_jpos_des)
            if not self._publish_command(cmd):
                return False
            
            expected_time = start_time + (i + 1) * self._sleep_duration
            sleep_time = expected_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if description:
            print(f"[Dex3] {description}完成")
        return True
    
    def initialize_hand(self, speed_factor: float = 1.0) -> bool:
        """
        初始化手部到自然位置 - 完全自动版
        
        参数:
            speed_factor: 速度因子 (>1加快, <1减慢)
        
        返回:
            bool: 是否成功
        """
        print(f"[Dex3-{self.hand}] 开始初始化灵巧手...")
        
        # 容错处理
        current_positions = self.get_current_joint_positions(timeout=2.0)
        if current_positions is None:
            print(f"[Dex3-{self.hand}] ⚠️ 无法读取当前位置，使用自然位作为起点")
            self._current_jpos_des = self._nature_pos.copy()
        else:
            # 异常检测与修正
            limits = self._get_joint_limits()
            for i in range(len(current_positions)):
                min_val, max_val = limits[i]
                if current_positions[i] is None or not (min_val <= current_positions[i] <= max_val):
                    current_positions[i] = self._nature_pos[i]
                    print(f"[Dex3-{self.hand}] ⚠️ 关节 {i} 已修正")
            self._current_jpos_des = current_positions.copy()
        
        # 🎯 使用 set_joint_positions (自动计算时间)
        return self.set_joint_positions(
            self._nature_pos,
            duration=None,  # 自动计算
            speed_factor=speed_factor
        )
    
    def stop_control(self) -> bool:
        """
        停止控制并返回自然位置
        
        🆕 使用 set_joint_positions 实现,自动计算时间
        
        返回:
            bool: 是否成功
        """
        print(f"[Dex3-{self.hand}] 停止控制...")
        
        # 🎯 直接使用 set_joint_positions,自动计算返回时间
        success = self.set_joint_positions(
            self._nature_pos,
            duration=None,  # 自动计算
            speed_factor=1.0
        )
        
        if not success:
            print(f"[Dex3-{self.hand}] 返回自然位失败")
            return False
        
        # 禁用所有电机
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            motor_cmds = [
                MotorCmd_(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=0)
                for _ in range(self.MOTOR_MAX)
            ]
            
            hand_cmd = HandCmd_(motor_cmd=motor_cmds, reserve=[0, 0, 0, 0])
            success = self._publish_command(hand_cmd)
            
            if success:
                print(f"[Dex3-{self.hand}] 控制已停止")
            return success
            
        except Exception as e:
            print(f"[Dex3-{self.hand}] 停止电机失败: {e}")
            return False
    
    def set_gesture(self, gesture_name: str) -> bool:
        """
        设置手势到预定义姿态
        
        Args:
            gesture_name: 手势名称 (nature, open等)
        
        Returns:
            bool: 是否成功
        """
        angles = Dex3Gestures.get_gesture(gesture_name, self.hand)
        if angles is None:
            return False
        
        return self.set_joint_positions(angles)
    
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
            hand.set_joint_positions(pose)
            
            # 指定时间
            hand.set_joint_positions(pose, duration=5.0)
            
            # 2倍速执行
            hand.set_joint_positions(pose, speed_factor=2.0)
        """
        if len(positions) != self.MOTOR_MAX:
            print(f"[Dex3] 错误: 位置数量({len(positions)})与关节数({self.MOTOR_MAX})不匹配")
            return False
        
        # 关节限位检查
        limits = self._get_joint_limits()
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
            duration = base_duration * 1.2 / speed_factor  # 20%余量 + 速度因子
            duration = max(duration, 0.5)  # 最小0.5秒
            
            print(f"[Dex3] 自动时间: {duration:.2f}s "
                  f"(Δ={max_delta:.3f}rad, 速度={speed_factor}x)")
        
        return self.smooth_transition(None, clamped_positions, duration, "")
    
    def get_current_joint_positions(self, timeout: float = 2.0) -> Optional[List[float]]:
        """
        获取当前关节位置
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            关节位置列表或None
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= self.MOTOR_MAX:
            try:
                return [float(ms.q) for ms in state.motor_state[:self.MOTOR_MAX]]
            except Exception as e:
                print(f"[Dex3] 解析关节位置失败: {e}")
        return None
    
    def get_joint_states(self, timeout: float = 2.0) -> Optional[Dict[str, Any]]:
        """
        获取详细的关节状态
        
        Returns:
            包含位置、速度、扭矩等信息的字典
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= self.MOTOR_MAX:
            try:
                joint_states = {
                    'positions': [],
                    'velocities': [],
                    'torques': []
                }
                for ms in state.motor_state[:self.MOTOR_MAX]:
                    joint_states['positions'].append(float(ms.q))
                    joint_states['velocities'].append(float(ms.dq))
                    joint_states['torques'].append(float(ms.tau_est))
                return joint_states
            except Exception as e:
                print(f"[Dex3] 解析关节状态失败: {e}")
        return None
    
    def get_pressure_data(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        获取触觉传感器数据
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            触觉传感器数据字典
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'press_sensor_state'):
            try:
                # 定义有效传感器索引
                useful_indices = {
                    'sensor_1': [3, 6, 8],
                    'sensor_3': [3, 6, 8],
                    'sensor_5': [3, 6, 8],
                    'sensor_0': [0, 2, 9, 11],
                    'sensor_2': [0, 2, 9, 11],
                    'sensor_4': [0, 2, 9, 11],
                    'sensor_6': [0, 2, 9, 11],
                    'sensor_7': [0, 2, 9, 11],
                    'sensor_8': [0, 2, 9, 11]
                }
                
                pressure_data = {}
                for i, sensor in enumerate(state.press_sensor_state):
                    sensor_key = f'sensor_{i}'
                    indices = useful_indices.get(sensor_key, [])
                    
                    pressure_data[sensor_key] = {
                        'pressure': [
                            sensor.pressure[idx] if idx in indices else None
                            for idx in range(len(sensor.pressure))
                        ],
                        'temperature': [
                            sensor.temperature[idx] if idx in indices else None
                            for idx in range(len(sensor.temperature))
                        ]
                    }
                
                return pressure_data
            except Exception as e:
                print(f"[Dex3] 解析压力数据失败: {e}")
        return None
    
    def get_imu_data(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        获取IMU数据
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            IMU数据字典
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'imu_state'):
            try:
                imu = state.imu_state
                return {
                    'quaternion': list(imu.quaternion),         # QwQxQyQz
                    'gyroscope': list(imu.gyroscope),           # 角速度 omega_xyz
                    'accelerometer': list(imu.accelerometer),   # 加速度 acc_xyz
                    'rpy': list(imu.rpy),                       # 欧拉角
                    'temperature': imu.temperature              # IMU温度
                }
            except Exception as e:
                print(f"[Dex3] 解析IMU数据失败: {e}")
        return None


class Dex3Gestures:
    """预定义手势库"""
    
    @staticmethod
    def get_gesture(gesture_name: str, hand_type: str = "right") -> Optional[List[float]]:
        """
        获取预定义手势的关节角度
        
        Args:
            gesture_name: 手势名称
            hand_type: 手的类型 ("left" 或 "right")
        
        Returns:
            7个关节角度列表（弧度）或None
        """
        if hand_type == "right":
            gestures = {
                "nature": [-0.029, -1.019, -1.667, 1.551, 1.702, 1.568, 1.710],
                "open": [-0.029, 0.587, 0.052, -0.053, -0.034, -0.022, -0.016],
                "press": [-0.030, 0.931, 1.575, -1.572, -1.719, -0.029, -0.016],
                "hello1": [-0.027, -1.022, -1.668, -0.059, -0.057, -0.040, -0.070]
            }
        else:  # left
            gestures = {
                "nature": [-0.028, 1.010, 1.511, -1.582, -1.779, -1.647, -1.827],
                "open": [0.005, -0.616, -0.085, -0.019, -0.035, -0.018, -0.025],
                "press": [-0.030, 0.931, 1.575, -1.572, -1.719, -0.029, -0.016],
                "hello1": [-0.027, 1.022, 1.668, 0.059, 0.057, 0.040, 0.070]
            }
        
        if gesture_name not in gestures:
            return None
        
        return gestures[gesture_name].copy()


@contextlib.contextmanager
def dex3_connection(hand="right", interface="eth0"):
    """Dex3连接上下文管理器"""
    dex3 = None
    try:
        dex3 = Dex3Client(hand=hand, interface=interface)
        yield dex3
    finally:
        if dex3:
            dex3.stop_control()