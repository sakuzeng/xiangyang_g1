import time
import math
import traceback
import logging
from .logger import setup_logger
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.dds.odometry_client import OdometryClient

# 配置日志
logger = setup_logger("advanced_locomotion")

class AdvancedLocomotionController:
    """
    高级底盘控制器
    功能：提供基于里程计反馈的精确移动和旋转能力
    """
    def __init__(self, interface="eth0"):
        self.interface = interface
        self.loco_client = None
        self.odom_client = None
        
        # 默认控制参数
        self.linear_velocity = 0.3
        self.angular_velocity = 0.50
        self.pos_tolerance = 0.05
        self.ang_tolerance = 0.08

    def initialize(self):
        """初始化底盘和里程计"""
        try:
            logger.info("📡 初始化里程计...")
            self.odom_client = OdometryClient(
                interface=self.interface,
                use_high_freq=False,
                use_low_freq=True
            )
            if not self.odom_client.initialize():
                logger.error("❌ 里程计初始化失败")
                return False
            
            # 等待数据稳定
            time.sleep(0.5)
            
            self.loco_client = LocoClient()
            self.loco_client.Init()
            
            logger.info("✅ 底盘运控初始化完成")
            return True
        except Exception as e:
            logger.error(f"❌ 底盘初始化异常: {e}")
            return False

    def move_forward_precise(self, distance: float):
        """基于里程计的精确前进/后退"""
        logger.info(f"🚶 精确移动 {distance:.2f}m")
        
        start_pos = self.odom_client.get_current_position()
        start_x, start_y = start_pos[0], start_pos[1]
        
        target_distance = abs(distance)
        direction = 1.0 if distance >= 0 else -1.0
        base_velocity = self.linear_velocity * direction
        
        max_time = target_distance / self.linear_velocity + 10
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                curr_pos = self.odom_client.get_current_position()
                curr_x, curr_y = curr_pos[0], curr_pos[1]
                
                moved = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
                remaining = target_distance - moved
                
                if remaining <= self.pos_tolerance:
                    break
                
                # 自适应速度（最后20cm减速）
                if remaining < 0.2:
                    velocity = base_velocity * max(0.3, remaining / 0.2)
                else:
                    velocity = base_velocity
                
                self.loco_client.Move(vx=velocity, vy=0.0, vyaw=0.0, continous_move=True)
                time.sleep(0.05)
            
            self.stop()
            
            # 打印结果
            final_pos = self.odom_client.get_current_position()
            actual_dist = math.sqrt((final_pos[0] - start_x)**2 + (final_pos[1] - start_y)**2)
            logger.info(f"✅ 移动完成: 目标={target_distance:.2f}m, 实际={actual_dist:.2f}m")
            
        except Exception as e:
            logger.error(f"❌ 移动异常: {e}")
        finally:
            self.stop()

    def turn_angle(self, angle_deg: float, direction: str = None):
        """
        基于里程计的精确旋转
        angle_deg: 角度 (度)
        direction: "left" 或 "right"，如果不填则根据 angle_deg 正负自动判断
        """
        target_angle_rad = math.radians(abs(angle_deg))
        
        # 确定方向
        if direction:
            is_left = (direction.lower() == "left")
        else:
            is_left = (angle_deg > 0)
            
        sign = 1 if is_left else -1
        target_delta = sign * target_angle_rad
        
        start_yaw = self.odom_client.get_current_yaw()
        logger.info(f"🔄 {'左转' if is_left else '右转'} {abs(angle_deg):.1f}°")
        
        max_time = target_angle_rad / self.angular_velocity + 10
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_time:
                curr_yaw = self.odom_client.get_current_yaw()
                
                # 计算当前相对于起始点的绝对角度变化 (归一化处理)
                current_diff = curr_yaw - start_yaw
                current_diff = math.atan2(math.sin(current_diff), math.cos(current_diff))
                
                remaining = target_delta - current_diff
                remaining = math.atan2(math.sin(remaining), math.cos(remaining))
                remaining_abs = abs(remaining)
                
                if remaining_abs <= self.ang_tolerance:
                    break
                
                # 过转保护
                if abs(current_diff) > target_angle_rad * 1.2:
                    logger.warning("⚠️ 检测到过转，强制停止")
                    break
                
                # 自适应角速度
                rot_dir = 1.0 if remaining > 0 else -1.0
                if remaining_abs < math.radians(30):
                    scale = max(0.6, remaining_abs / math.radians(30))
                    omega = self.angular_velocity * scale * rot_dir
                else:
                    omega = self.angular_velocity * rot_dir
                
                self.loco_client.Move(vx=0.0, vy=0.0, vyaw=omega, continous_move=True)
                time.sleep(0.05)
            
            self.stop()
            
            # 验证结果
            time.sleep(0.5) # 等待完全静止更新里程计
            final_yaw = self.odom_client.get_current_yaw()
            actual_change = math.degrees(math.atan2(math.sin(final_yaw - start_yaw), math.cos(final_yaw - start_yaw)))
            logger.info(f"✅ 旋转完成: 实际变化 {actual_change:.1f}°")
            
        except Exception as e:
            logger.error(f"❌ 旋转异常: {e}")
        finally:
            self.stop()

    def stop(self):
        """停止移动"""
        if self.loco_client:
            self.loco_client.StopMove()
            time.sleep(0.3)

    def cleanup(self):
        self.stop()
        if self.odom_client:
            self.odom_client.print_stats()