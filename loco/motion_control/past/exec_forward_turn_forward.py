import sys
import os
import time
import math
import traceback
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.dds.odometry_client import OdometryClient

class SimpleMover:
    """简易移动控制 - 前进 -> 左转 -> 前进"""
    
    def __init__(self, interface="eth0", first_distance=0.6):
        self.interface = interface
        self.first_distance = first_distance  # 第一段前进的距离
        
        # 控制参数
        self.LINEAR_VELOCITY = 0.3      # 线速度(m/s)
        self.ANGULAR_VELOCITY = 0.50    # 角速度(rad/s)
        self.POSITION_TOLERANCE = 0.05  # 位置容差(m)
        self.ANGLE_TOLERANCE = 0.08     # 角度容差(rad)
        
        self.loco_client = None
        self.odom_client = None
    
    def initialize(self):
        """初始化底盘和里程计"""
        try:
            ChannelFactoryInitialize(0, self.interface)
            
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
            
            self.loco_client = LocoClient()
            self.loco_client.Init()
            
            print("✅ 初始化完成\n")
            return True
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False

    def move_distance(self, distance: float):
        """前进/后退指定距离"""
        direction = 1 if distance > 0 else -1
        target_distance = abs(distance)
        
        # 获取起始位置
        start_pos = self.odom_client.get_current_position()
        start_x, start_y = start_pos[0], start_pos[1]
        
        print(f"{'🚶 前进' if direction > 0 else '🚶 后退'} {target_distance:.2f}m")
        
        base_velocity = self.LINEAR_VELOCITY * direction
        max_time = target_distance / abs(self.LINEAR_VELOCITY) + 5
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                curr_pos = self.odom_client.get_current_position()
                curr_x, curr_y = curr_pos[0], curr_pos[1]
                
                moved = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
                remaining = target_distance - moved
                
                if remaining <= self.POSITION_TOLERANCE:
                    break
                
                # 自适应减速
                if remaining < 0.2:
                    velocity = base_velocity * max(0.3, remaining / 0.2)
                else:
                    velocity = base_velocity
                
                self.loco_client.Move(vx=velocity, vy=0.0, vyaw=0.0, continous_move=True)
                time.sleep(0.05)
            
            self.loco_client.StopMove()
            time.sleep(0.3)
            
            # 打印结果
            final_pos = self.odom_client.get_current_position()
            actual = math.sqrt((final_pos[0]-start_x)**2 + (final_pos[1]-start_y)**2)
            print(f"✅ 完成: 目标={target_distance:.2f}m, 实际={actual:.2f}m\n")
            
        except Exception as e:
            print(f"❌ 移动异常: {e}")
            self.loco_client.StopMove()

    def turn_left_90(self):
        """向左转90度 (使用绝对角度差控制，带开环补偿)"""
        # 🔧 开环补偿：由于实际总是转不到90度（只有70几度），人为增加目标角度
        # 目标设为 110 度，期望实际能转到 90 度左右
        target_angle = math.radians(110) 
        print(f"🔄 左转 90° (内部目标补偿为 110°)")
        
        start_yaw = self.odom_client.get_current_yaw()
        target_yaw_diff = target_angle  # 目标变化量
        
        omega = self.ANGULAR_VELOCITY
        max_time = target_angle / self.ANGULAR_VELOCITY + 5
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
                
                # 自适应减速
                if abs(remaining) < math.radians(30):
                    scale = max(0.4, abs(remaining) / math.radians(30))
                    current_omega = omega * scale
                else:
                    current_omega = omega
                
                # 始终保持向左转 (omega为正)
                self.loco_client.Move(vx=0.0, vy=0.0, vyaw=current_omega, continous_move=True)
                time.sleep(0.05)
            
            self.loco_client.StopMove()
            time.sleep(0.8)
            
            # 结果验证
            final_yaw = self.odom_client.get_current_yaw()
            final_delta = final_yaw - start_yaw
            final_delta = math.atan2(math.sin(final_delta), math.cos(final_delta))
            error_deg = math.degrees(abs(target_yaw_diff - final_delta))
            print(f"✅ 第一阶段: 实际转过 {math.degrees(final_delta):.1f}°, 误差 {error_deg:.1f}°")
            
            print()
            
        except Exception as e:
            print(f"❌ 旋转异常: {e}")
            self.loco_client.StopMove()

    def run(self):
        """执行任务序列"""
        print("="*50)
        print(f"🚀 开始任务: 前进{self.first_distance}m -> 左转90° -> 前进1m")
        print("="*50 + "\n")
        
        try:
            # 1. 前进指定距离
            self.move_distance(self.first_distance)
            
            # 2. 左转90度
            self.turn_left_90()
            
            # 3. 前进1米
            self.move_distance(0.6)
            
            print("✨ 全部任务完成")
            
        except KeyboardInterrupt:
            print("\n⚠️ 用户中断")
        finally:
            if self.loco_client:
                self.loco_client.StopMove()

def main():
    if len(sys.argv) < 2:
        interface = "eth0"
    else:
        interface = sys.argv[1]
    
    # 可以在这里修改第一段前进的距离，默认为0.6米
    mover = SimpleMover(interface=interface, first_distance=3)
    
    if mover.initialize():
        mover.run()

if __name__ == "__main__":
    main()