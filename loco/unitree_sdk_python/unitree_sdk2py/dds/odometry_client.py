"""
里程计客户端 - 订阅机器人位置、速度、姿态信息
支持高频(500Hz)和低频(20Hz)两种频率
"""
import time
import threading
from typing import Optional, Callable
from dataclasses import dataclass
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


@dataclass
class OdometryData:
    """里程计数据结构"""
    # 位置 (世界坐标系, m)
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    
    # 速度 (机器人坐标系, m/s)
    vel_x: float = 0.0
    vel_y: float = 0.0
    vel_z: float = 0.0
    
    # 欧拉角 (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # yaw角速度 (rad/s)
    yaw_speed: float = 0.0
    
    # 四元数
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    
    # 时间戳
    timestamp: float = 0.0
    
    def __str__(self):
        return (
            f"Position: ({self.pos_x:.3f}, {self.pos_y:.3f}, {self.pos_z:.3f}) m\n"
            f"Velocity: ({self.vel_x:.3f}, {self.vel_y:.3f}, {self.vel_z:.3f}) m/s\n"
            f"Euler: ({self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}) rad\n"
            f"Yaw Speed: {self.yaw_speed:.3f} rad/s"
        )


class OdometryClient:
    """里程计客户端"""
    
    # DDS话题
    TOPIC_HIGH_FREQ = "rt/odommodestate"      # 500Hz
    TOPIC_LOW_FREQ = "rt/lf/odommodestate"    # 20Hz
    
    def __init__(self, interface: str = "eth0", use_high_freq: bool = True, use_low_freq: bool = False):
        """
        初始化里程计客户端
        
        参数:
            interface: 网络接口
            use_high_freq: 是否订阅高频数据(500Hz)
            use_low_freq: 是否订阅低频数据(20Hz)
        """
        self.interface = interface
        self.use_high_freq = use_high_freq
        self.use_low_freq = use_low_freq
        
        # 数据存储
        self.high_freq_data = OdometryData()
        self.low_freq_data = OdometryData()
        
        # 订阅器
        self.high_freq_sub: Optional[ChannelSubscriber] = None
        self.low_freq_sub: Optional[ChannelSubscriber] = None
        
        # 回调函数
        self.high_freq_callback: Optional[Callable] = None
        self.low_freq_callback: Optional[Callable] = None
        
        # 线程锁
        self.high_freq_lock = threading.Lock()
        self.low_freq_lock = threading.Lock()
        
        # 统计
        self.high_freq_count = 0
        self.low_freq_count = 0
        
        print(f"📡 里程计客户端初始化")
        print(f"   接口: {interface}")
        print(f"   高频(500Hz): {'✅' if use_high_freq else '❌'}")
        print(f"   低频(20Hz): {'✅' if use_low_freq else '❌'}")
    
    def initialize(self) -> bool:
        """初始化订阅器"""
        try:
            # 初始化DDS通道（如果还没初始化）
            try:
                ChannelFactoryInitialize(0, self.interface)
            except:
                pass  # 可能已经初始化过
            
            # 创建高频订阅器
            if self.use_high_freq:
                self.high_freq_sub = ChannelSubscriber(self.TOPIC_HIGH_FREQ, SportModeState_)
                self.high_freq_sub.Init(self._high_freq_handler, 1)
                print(f"✅ 高频订阅器已创建: {self.TOPIC_HIGH_FREQ}")
            
            # 创建低频订阅器
            if self.use_low_freq:
                self.low_freq_sub = ChannelSubscriber(self.TOPIC_LOW_FREQ, SportModeState_)
                self.low_freq_sub.Init(self._low_freq_handler, 1)
                print(f"✅ 低频订阅器已创建: {self.TOPIC_LOW_FREQ}")
            
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def _parse_state(self, msg: SportModeState_) -> OdometryData:
        """解析SportModeState消息"""
        data = OdometryData()
        
        # 位置
        data.pos_x = msg.position[0]
        data.pos_y = msg.position[1]
        data.pos_z = msg.position[2]
        
        # 速度
        data.vel_x = msg.velocity[0]
        data.vel_y = msg.velocity[1]
        data.vel_z = msg.velocity[2]
        
        # 欧拉角
        data.roll = msg.imu_state.rpy[0]
        data.pitch = msg.imu_state.rpy[1]
        data.yaw = msg.imu_state.rpy[2]
        
        # yaw角速度
        data.yaw_speed = msg.yaw_speed
        
        # 四元数
        data.quat_w = msg.imu_state.quaternion[0]
        data.quat_x = msg.imu_state.quaternion[1]
        data.quat_y = msg.imu_state.quaternion[2]
        data.quat_z = msg.imu_state.quaternion[3]
        
        # 时间戳
        data.timestamp = time.time()
        
        return data
    
    def _high_freq_handler(self, msg: SportModeState_):
        """高频消息处理"""
        with self.high_freq_lock:
            self.high_freq_data = self._parse_state(msg)
            self.high_freq_count += 1
        
        # 调用用户回调
        if self.high_freq_callback:
            self.high_freq_callback(self.high_freq_data)
    
    def _low_freq_handler(self, msg: SportModeState_):
        """低频消息处理"""
        with self.low_freq_lock:
            self.low_freq_data = self._parse_state(msg)
            self.low_freq_count += 1
        
        # 调用用户回调
        if self.low_freq_callback:
            self.low_freq_callback(self.low_freq_data)
    
    def get_high_freq_data(self) -> OdometryData:
        """获取高频数据（线程安全）"""
        with self.high_freq_lock:
            return self.high_freq_data
    
    def get_low_freq_data(self) -> OdometryData:
        """获取低频数据（线程安全）"""
        with self.low_freq_lock:
            return self.low_freq_data
    
    def set_high_freq_callback(self, callback: Callable[[OdometryData], None]):
        """设置高频数据回调"""
        self.high_freq_callback = callback
    
    def set_low_freq_callback(self, callback: Callable[[OdometryData], None]):
        """设置低频数据回调"""
        self.low_freq_callback = callback
    
    def get_current_position(self) -> tuple:
        """获取当前位置 (x, y, z)"""
        data = self.get_high_freq_data() if self.use_high_freq else self.get_low_freq_data()
        return (data.pos_x, data.pos_y, data.pos_z)
    
    def get_current_yaw(self) -> float:
        """获取当前yaw角"""
        data = self.get_high_freq_data() if self.use_high_freq else self.get_low_freq_data()
        return data.yaw
    
    def print_stats(self):
        """打印统计信息"""
        print(f"\n📊 里程计统计:")
        if self.use_high_freq:
            print(f"   高频消息: {self.high_freq_count}")
        if self.use_low_freq:
            print(f"   低频消息: {self.low_freq_count}")


# 示例用法
def main():
    """测试程序"""
    import sys
    
    interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    print("="*70)
    print("📡 里程计客户端测试")
    print("="*70)
    
    # 创建客户端（订阅低频数据，便于观察）
    client = OdometryClient(interface=interface, use_high_freq=False, use_low_freq=True)
    
    # 设置回调（可选）
    def on_data_received(data: OdometryData):
        print(f"\n📍 新数据:")
        print(data)
        print("-"*70)
    
    client.set_low_freq_callback(on_data_received)
    
    # 初始化
    if not client.initialize():
        print("❌ 初始化失败")
        sys.exit(1)
    
    print("\n✅ 开始接收数据 (Ctrl+C退出)...")
    
    try:
        while True:
            time.sleep(1)
            
            # 手动获取数据示例
            pos = client.get_current_position()
            yaw = client.get_current_yaw()
            print(f"当前位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), Yaw: {yaw:.3f}")
            
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断")
    finally:
        client.print_stats()


if __name__ == "__main__":
    main()