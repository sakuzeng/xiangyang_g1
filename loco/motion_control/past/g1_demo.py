import sys
import time
import signal
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.arm.arm_client import G1ArmClient, G1ArmConfig
from unitree_sdk2py.dex3.dex3_client import Dex3Client

import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from common.robot_state_manager import robot_state

# 预定义手臂位姿数据
ARM_POSES = {
    "nature": [0.243, 0.173, -0.016, 0.796, 0.090, 0.027, -0.008, 0.250, -0.175, 0.025, 0.801, -0.111, 0.035, 0.009],
    "hello1": [0.243, 0.173, -0.016, 0.796, 0.090, 0.027, -0.008,
               -0.567, -0.226, -0.418, -0.150, -1.308, 0.003, -0.315],
    "hello2": [0.243, 0.173, -0.016, 0.796, 0.090, 0.027, -0.008,
               -0.567, -0.226, -0.787, -0.073, -1.141, 0.064, -0.161],
    "hello3": [0.243, 0.173, -0.016, 0.796, 0.090, 0.027, -0.008, 
               -0.567, -0.226, 0.137, -0.257, -1.615, -0.112, -0.189],
}

# 预定义灵巧手位姿数据
HAND_POSES = {
    "nature": [-0.029, -1.019, -1.667, 1.551, 1.702, 1.568, 1.710],
    "hello1": [-0.027, -1.022, -1.668, -0.059, -0.057, -0.040, -0.070],
}

class KeyboardRobotControl:
    """G1机器人键盘控制类 - 使用状态管理器（左右分离版）"""
    
    def __init__(self, interface_name="eth0"):
        self.interface_name = interface_name
        self.loco_client = None
        self.arm_client = None
        self.left_hand_client = None
        self.right_hand_client = None
        self.is_arm_hand_initialized = False
        self.cleanup_executed = False 

    def initialize(self):
        """初始化所有机器人控制模块"""
        try:
            ChannelFactoryInitialize(0, self.interface_name)
            
            # 初始化运动控制
            print("🦿 初始化运动控制模块...")
            self.loco_client = LocoClient()
            self.loco_client.Init()
            print("✅ 运动控制模块初始化成功")
            
            # 🆕 初始化手臂（使用状态管理器获取单例）
            print("🦾 初始化手臂控制模块...")
            self.arm_client = robot_state.get_or_create_arm_client(self.interface_name)
            print("✅ 手臂控制模块初始化成功")
            
            # 🆕 初始化灵巧手（左右分离）
            print("🤲 初始化左手灵巧手控制模块...")
            self.left_hand_client = robot_state.get_or_create_hand_client(
                hand="left", 
                interface=self.interface_name
            )
            print("✅ 左手灵巧手控制模块初始化成功")
            
            print("🤲 初始化右手灵巧手控制模块...")
            self.right_hand_client = robot_state.get_or_create_hand_client(
                hand="right", 
                interface=self.interface_name
            )
            print("✅ 右手灵巧手控制模块初始化成功")
            
            # 自动初始化到安全位姿
            print("\n🤖 正在初始化手臂和灵巧手到自然位姿...")
            if not self.initialize_arm_and_hand():
                print("⚠️ 手臂/灵巧手初始化失败")
                return False
            
            print("\n✅ 所有控制模块初始化完成")
            print(f"📊 当前状态: {robot_state.get_status_string()}")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            raise

    def execute_forward_movement(self):
        """执行前进1米动作 - 使用状态管理"""
        try:
            print(f"\n📊 运动前状态: {robot_state.get_status_string()}")
            
            # 🆕 使用状态管理器检查
            if robot_state.is_any_limb_controlling():
                print("⚠️ 检测到手臂/灵巧手正在控制，需要先停止...")
                if not robot_state.emergency_stop_all():
                    print("❌ 无法停止手臂/灵巧手，中止移动")
                    return
                print("✅ 手臂/灵巧手已停止，可以安全移动")
                time.sleep(0.5)
            
            print("🚶 开始执行前进1米动作...")
            self.loco_client.SetVelocity(vx=0.5, vy=0.0, omega=0.0, duration=2.0)
            time.sleep(2.5)
            self.loco_client.StopMove()
            time.sleep(0.5)
            
            print("✅ 前进1米动作执行完成")
            print(f"📊 运动后状态: {robot_state.get_status_string()}")
            
        except Exception as e:
            print(f"❌ 前进动作执行失败: {e}")

    def execute_backward_movement(self):
        """执行后退1米动作"""
        try:
            print(f"\n📊 运动前状态: {robot_state.get_status_string()}")
            
            if robot_state.is_any_limb_controlling():
                print("⚠️ 检测到手臂/灵巧手正在控制，需要先停止...")
                if not robot_state.emergency_stop_all():
                    print("❌ 无法停止手臂/灵巧手，中止移动")
                    return
                time.sleep(0.5)
            
            print("🚶 开始执行后退1米动作...")
            self.loco_client.SetVelocity(vx=-0.5, vy=0.0, omega=0.0, duration=2.0)
            time.sleep(2.5)
            self.loco_client.StopMove()
            time.sleep(0.5)
            
            print("✅ 后退1米动作执行完成")
            print(f"📊 运动后状态: {robot_state.get_status_string()}")
            
        except Exception as e:
            print(f"❌ 后退动作执行失败: {e}")

    def initialize_arm_and_hand(self):
        """初始化手臂和灵巧手到自然位姿 - 🆕 使用左右分离的状态管理"""
        try:
            if self.is_arm_hand_initialized:
                return True
            
            print("🤖 初始化手臂和灵巧手...")
            
            # 🆕 使用双臂上下文管理器
            with robot_state.safe_dual_arm_control(source="initialization"):
                if not self.arm_client.initialize_arms():
                    print("❌ 手臂初始化失败")
                    return False
            
            # 🆕 分别初始化左右手（使用左右分离的上下文）
            with robot_state.safe_hand_control(hand="left", source="initialization"):
                if not self.left_hand_client.initialize_hand():
                    print("❌ 左手灵巧手初始化失败")
                    return False
            
            with robot_state.safe_hand_control(hand="right", source="initialization"):
                if not self.right_hand_client.initialize_hand():
                    print("❌ 右手灵巧手初始化失败")
                    return False
            
            self.is_arm_hand_initialized = True
            print("✅ 手臂和灵巧手初始化完成")
            return True
            
        except Exception as e:
            print(f"❌ 手臂和灵巧手初始化失败: {e}")
            return False

    def execute_hello_gesture(self):
        """执行打招呼动作序列 - 🆕 使用右手臂+右手"""
        try:
            print(f"\n📊 动作前状态: {robot_state.get_status_string()}")
            print("👋 开始执行打招呼动作...")
            
            if not self.is_arm_hand_initialized:
                if not self.initialize_arm_and_hand():
                    return False
            
            # 🆕 使用右手臂上下文管理器（指定 arm='right'）
            print("📍 步骤1: 右手臂移动到 hello1 位姿")
            with robot_state.safe_arm_control(arm="right", source="hello_gesture"):
                self.arm_client.set_joint_positions(ARM_POSES["hello1"])
            
            # 🆕 使用右手上下文管理器（指定 hand='right'）
            print("🤲 步骤2: 右手灵巧手移动到 hello1 位姿")
            with robot_state.safe_hand_control(hand="right", source="hello_gesture"):
                self.right_hand_client.set_joint_positions(HAND_POSES["hello1"])
            
            print("📍 步骤3-5: 连续挥手动作")
            with robot_state.safe_arm_control(arm="right", source="hello_gesture"):
                self.arm_client.set_joint_positions(ARM_POSES["hello2"])
                self.arm_client.set_joint_positions(ARM_POSES["hello3"])
                self.arm_client.set_joint_positions(ARM_POSES["hello2"])
            
            print("🔄 步骤6: 恢复到自然位姿")
            with robot_state.safe_hand_control(hand="right", source="hello_gesture"):
                self.right_hand_client.set_joint_positions(HAND_POSES["nature"])
            
            with robot_state.safe_arm_control(arm="right", source="hello_gesture"):
                self.arm_client.set_joint_positions(ARM_POSES["nature"])
            
            print("✅ 打招呼动作执行完成")
            print(f"📊 动作后状态: {robot_state.get_status_string()}")
            return True
            
        except Exception as e:
            print(f"❌ 打招呼动作执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def emergency_stop_arm_hand(self):
        """紧急停止 - 使用状态管理器"""
        if self.cleanup_executed:
            return
        self.cleanup_executed = True
        
        print("🚨 执行紧急停止...")
        robot_state.emergency_stop_all()  # 🆕 使用状态管理器的统一停止
        print("✅ 紧急停止完成")

    def run_loop(self):
        """主输入循环"""
        print("\n" + "="*60)
        print("⌨️  G1 机器人键盘控制终端 (左右分离状态管理)")
        print("="*60)
        print("指令列表:")
        print("  [1] -> 前进 1 米")
        print("  [2] -> 后退 1 米")
        print("  [3] -> 打招呼 (右手臂+右手)")
        print("  [s] -> 显示当前状态")
        print("  [q] -> 退出程序")
        print("="*60)

        while True:
            try:
                cmd = input(f"\n请输入指令 (当前: {robot_state.get_status_string()}): ").strip().lower()

                if cmd == '1':
                    self.execute_forward_movement()
                elif cmd == '2':
                    self.execute_backward_movement()
                elif cmd == '3':
                    self.execute_hello_gesture()
                elif cmd == 's':
                    print("\n📊 详细状态:")
                    print(f"  左臂: {robot_state.get_arm_status('left')}")
                    print(f"  右臂: {robot_state.get_arm_status('right')}")
                    print(f"  左手: {robot_state.get_hand_status('left')}")
                    print(f"  右手: {robot_state.get_hand_status('right')}")
                elif cmd == 'q':
                    print("正在退出...")
                    break
                else:
                    print("⚠️ 无效指令，请输入 1, 2, 3, s 或 q")
                    
            except KeyboardInterrupt:
                print("\n检测到中断...")
                break
        
        self.cleanup()

    def cleanup(self):
        """资源清理"""
        if not self.cleanup_executed:
            self.emergency_stop_arm_hand()
        robot_state.reset_all_states()
        print("✅ 程序已安全退出")

# 全局变量用于信号处理
demo = None

def signal_handler(signum, frame):
    """处理 Ctrl+C"""
    print("\n🛑 接收到退出信号")
    if demo:
        demo.cleanup()
    sys.exit(0)

def main():
    global demo
    
    interface_name = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    print(f"🔧 使用网络接口: {interface_name}")
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    demo = KeyboardRobotControl(interface_name)
    
    try:
        if demo.initialize():
            demo.run_loop()
    except Exception as e:
        print(f"❌ 运行错误: {e}")
        demo.cleanup()

if __name__ == "__main__":
    main()