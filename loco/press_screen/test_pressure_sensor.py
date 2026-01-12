#!/usr/bin/env python3
"""
Dex3 灵巧手压力传感器测试程序 - 集成状态管理
"""
import sys
import time
import threading
from typing import Optional, Dict, Any, List

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.dex3.dex3_client import Dex3Client

# 🆕 导入状态管理器（可选）
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from common.robot_state_manager import robot_state
    STATE_MANAGER_AVAILABLE = True
except ImportError:
    STATE_MANAGER_AVAILABLE = False
    print("⚠️  状态管理器不可用，独立运行模式")


class PressureSensorTester:
    """压力传感器测试器 - 集成状态管理"""
    
    # 手指和关节的详细映射
    FINGER_SENSOR_MAP = {
        'thumb': {
            'name': '拇指',
            'joints': {
                'tip': {'sensor_id': 1, 'name': '指尖', 'indices': [3, 6, 8]},
                'base': {'sensor_id': 0, 'name': '基部', 'indices': [0, 2, 9, 11]}
            }
        },
        'index': {
            'name': '食指',
            'joints': {
                'tip': {'sensor_id': 5, 'name': '指尖', 'indices': [3, 6, 8]},
                'base': {'sensor_id': 4, 'name': '基部', 'indices': [0, 2, 9, 11]}
            }
        },
        'middle': {
            'name': '中指',
            'joints': {
                'tip': {'sensor_id': 3, 'name': '指尖', 'indices': [3, 6, 8]},
                'base': {'sensor_id': 2, 'name': '基部', 'indices': [0, 2, 9, 11]}
            }
        },
        'palm': {
            'name': '手掌',
            'joints': {
                'area_1': {'sensor_id': 6, 'name': '区域1', 'indices': [0, 2, 9, 11]},
                'area_2': {'sensor_id': 7, 'name': '区域2', 'indices': [0, 2, 9, 11]},
                'area_3': {'sensor_id': 8, 'name': '区域3', 'indices': [0, 2, 9, 11]}
            }
        }
    }
    
    def __init__(self, hand: str = "left", interface: str = "eth0", read_only: bool = False):
        """
        初始化测试器
        
        Args:
            hand: 手的类型 ("left" 或 "right")
            interface: 网络接口
            read_only: 只读模式（不初始化控制，只读取传感器数据）
        """
        self.hand = hand
        self.interface = interface
        self.read_only = read_only  # 🆕 只读模式标志
        self.dex3_client: Optional[Dex3Client] = None
        
        # 压力阈值配置
        self.pressure_threshold = 100000.0  # 原始值阈值
        self.temperature_threshold = 40.0   # 温度阈值(摄氏度)
        
        # 当前监控的传感器
        self.selected_sensors: List[Dict[str, Any]] = []
        
        # 旧的映射保留用于兼容
        self.sensor_mapping = {
            'thumb_tip': 1, 'index_tip': 5, 'middle_tip': 3,
            'thumb_base': 0, 'index_base': 4, 'middle_base': 2,
            'palm_1': 6, 'palm_2': 7, 'palm_3': 8,
        }
        
        self.useful_indices = {
            'sensor_1': [3, 6, 8], 'sensor_3': [3, 6, 8], 'sensor_5': [3, 6, 8],
            'sensor_0': [0, 2, 9, 11], 'sensor_2': [0, 2, 9, 11], 'sensor_4': [0, 2, 9, 11],
            'sensor_6': [0, 2, 9, 11], 'sensor_7': [0, 2, 9, 11], 'sensor_8': [0, 2, 9, 11]
        }
        
        self._monitoring = False
        self._monitor_thread: Optional[threading.Thread] = None
    
    def initialize(self) -> bool:
        """初始化灵巧手 - 集成状态管理"""
        try:
            if self.read_only:
                print(f"🔧 初始化 {self.hand} 手传感器读取（只读模式）...")
                ChannelFactoryInitialize(0, self.interface)
                
                self.dex3_client = Dex3Client(hand=self.hand, interface=self.interface)
                
                # 🆕 只读模式也可以注册（但不设置控制状态）
                if STATE_MANAGER_AVAILABLE:
                    robot_state.register_hand_client(self.dex3_client)
                    print("📊 已注册到状态管理器（只读模式）")
                
                print("✅ 传感器读取初始化成功（灵巧手不会被激活）")
                return True
            else:
                print(f"🔧 初始化 {self.hand} 手灵巧手（完整模式）...")
                ChannelFactoryInitialize(0, self.interface)
                
                self.dex3_client = Dex3Client(hand=self.hand, interface=self.interface)
                
                # 🆕 注册到状态管理器
                if STATE_MANAGER_AVAILABLE:
                    robot_state.register_hand_client(self.dex3_client)
                
                if not self.dex3_client.initialize_hand():
                    print("❌ 灵巧手初始化失败")
                    return False
                
                # 🆕 标记控制中
                if STATE_MANAGER_AVAILABLE:
                    robot_state.set_hand_controlling(True, source="pressure_sensor_test")
                    print(f"📊 当前状态: {robot_state.get_status_string()}")
                
                print("✅ 灵巧手初始化成功（控制已激活）")
                return True
                
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def display_sensor_menu(self) -> bool:
        """
        显示传感器选择菜单
        
        Returns:
            是否成功选择传感器
        """
        print("\n" + "="*60)
        print(f"🖐️  {self.hand.upper()} 手 - 传感器选择")
        print("="*60)
        
        # 显示所有可用的手指和关节
        finger_options = []
        index = 1
        
        for finger_key, finger_info in self.FINGER_SENSOR_MAP.items():
            print(f"\n【{finger_info['name']}】")
            for joint_key, joint_info in finger_info['joints'].items():
                option_text = f"  {index}. {finger_info['name']}-{joint_info['name']} (sensor_{joint_info['sensor_id']}, {len(joint_info['indices'])}个点位)"
                print(option_text)
                finger_options.append({
                    'finger': finger_key,
                    'finger_name': finger_info['name'],
                    'joint': joint_key,
                    'joint_name': joint_info['name'],
                    'sensor_id': joint_info['sensor_id'],
                    'indices': joint_info['indices']
                })
                index += 1
        
        print("\n" + "="*60)
        print("💡 选择方式:")
        print("  - 单个传感器: 输入数字 (如: 1)")
        print("  - 多个传感器: 用逗号分隔 (如: 1,3,5)")
        print("  - 全部传感器: 输入 'all'")
        print("  - 取消选择: 输入 'q'")
        print("="*60)
        
        choice = input("\n请选择要监控的传感器: ").strip()
        
        if choice.lower() == 'q':
            return False
        
        self.selected_sensors = []
        
        if choice.lower() == 'all':
            self.selected_sensors = finger_options
            print(f"✅ 已选择全部 {len(self.selected_sensors)} 个传感器")
        else:
            try:
                selected_indices = [int(x.strip()) for x in choice.split(',')]
                for idx in selected_indices:
                    if 1 <= idx <= len(finger_options):
                        self.selected_sensors.append(finger_options[idx - 1])
                    else:
                        print(f"⚠️  无效选项: {idx}")
                
                if self.selected_sensors:
                    print(f"✅ 已选择 {len(self.selected_sensors)} 个传感器:")
                    for sensor in self.selected_sensors:
                        print(f"  - {sensor['finger_name']}-{sensor['joint_name']}")
                else:
                    print("❌ 未选择任何传感器")
                    return False
                    
            except ValueError:
                print("❌ 输入格式错误")
                return False
        
        return True
    
    def read_pressure_once(self) -> Optional[Dict[str, Any]]:
        """读取一次压力数据"""
        if self.dex3_client is None:
            print("❌ 灵巧手未初始化")
            return None
        
        pressure_data = self.dex3_client.get_pressure_data(timeout=1.0)
        return pressure_data
    
    def print_selected_sensor_data(self, pressure_data: Dict[str, Any]):
        """打印选中传感器的压力数据"""
        if pressure_data is None:
            print("❌ 无压力数据")
            return
        
        if not self.selected_sensors:
            print("⚠️  未选择任何传感器")
            return
        
        print("\n" + "="*60)
        print(f"📊 选中传感器压力数据 ({self.hand.upper()} 手)")
        print("="*60)
        
        for sensor_info in self.selected_sensors:
            sensor_key = f"sensor_{sensor_info['sensor_id']}"
            sensor_data = pressure_data.get(sensor_key, {})
            
            print(f"\n🔹 {sensor_info['finger_name']}-{sensor_info['joint_name']} ({sensor_key}):")
            
            pressures = sensor_data.get('pressure', [])
            valid_indices = sensor_info['indices']
            
            valid_pressures = [
                pressures[idx] for idx in valid_indices 
                if idx < len(pressures) and pressures[idx] is not None
            ]
            
            if valid_pressures:
                display_pressures = [p / 10000.0 for p in valid_pressures]
                max_pressure = max(display_pressures)
                avg_pressure = sum(display_pressures) / len(display_pressures)
                
                print(f"   压力: 最大={max_pressure:.2f}, 平均={avg_pressure:.2f} (单位:10^4)")
                
                threshold_display = self.pressure_threshold / 10000.0
                if max_pressure > threshold_display:
                    print(f"   ⚠️  压力超过阈值 ({threshold_display:.2f})!")
                
                print(f"   有效点位: {valid_indices}")
                for i, idx in enumerate(valid_indices):
                    if idx < len(pressures) and pressures[idx] is not None:
                        display_val = pressures[idx] / 10000.0
                        print(f"     点位[{idx}]: {display_val:6.2f}")
            else:
                print("   压力: 无有效数据")
            
            temperatures = sensor_data.get('temperature', [])
            valid_temps = [
                temperatures[idx] for idx in valid_indices
                if idx < len(temperatures) and temperatures[idx] is not None
            ]
            
            if valid_temps:
                avg_temp = sum(valid_temps) / len(valid_temps)
                print(f"   温度: 平均={avg_temp:.2f}°C")
        
        print("="*60 + "\n")
    
    def monitor_selected_sensors(self, duration: float = 10.0, interval: float = 0.5):
        """监控选中的传感器"""
        if not self.selected_sensors:
            print("⚠️  未选择任何传感器")
            return
        
        print(f"\n🔍 开始监控选中传感器 ({duration}秒, 每{interval}秒采样一次)...")
        print(f"📍 监控传感器列表:")
        for sensor in self.selected_sensors:
            print(f"  - {sensor['finger_name']}-{sensor['joint_name']}")
        print()
        
        start_time = time.time()
        sample_count = 0
        
        while time.time() - start_time < duration:
            sample_count += 1
            elapsed = time.time() - start_time
            print(f"\n⏱️  采样 #{sample_count} (时间: {elapsed:.1f}s)")
            
            pressure_data = self.read_pressure_once()
            
            if pressure_data:
                # 简洁输出
                for sensor_info in self.selected_sensors:
                    sensor_key = f"sensor_{sensor_info['sensor_id']}"
                    sensor_data = pressure_data.get(sensor_key, {})
                    pressures = sensor_data.get('pressure', [])
                    
                    valid_pressures = [
                        pressures[idx] for idx in sensor_info['indices']
                        if idx < len(pressures) and pressures[idx] is not None
                    ]
                    
                    if valid_pressures:
                        max_p_display = max(valid_pressures) / 10000.0
                        status = '🔴 按下' if max_p_display > self.pressure_threshold / 10000.0 else '⚪ 未按'
                        print(f"  {sensor_info['finger_name']}-{sensor_info['joint_name']}: {max_p_display:6.2f} (10^4) {status}")
            else:
                print("⚠️  无法读取压力数据")
            
            time.sleep(interval)
        
        print(f"\n✅ 监控完成 (共采样 {sample_count} 次)")
    
    def read_pressure_data(self, pressure_data: Dict[str, Any]):
        """打印压力数据(格式化输出) - 保留用于全面显示"""
        if pressure_data is None:
            print("❌ 无压力数据")
            return
        
        print("\n" + "="*60)
        print("📊 压力传感器数据")
        print("="*60)
        
        for sensor_key, sensor_info in pressure_data.items():
            sensor_idx = int(sensor_key.split('_')[1])
            
            sensor_name = "Unknown"
            for name, idx in self.sensor_mapping.items():
                if idx == sensor_idx:
                    sensor_name = name
                    break
            
            print(f"\n🔹 {sensor_key} ({sensor_name}):")
            
            valid_indices = self.useful_indices.get(sensor_key, [])
            pressures = sensor_info.get('pressure', [])
            
            valid_pressures = [
                pressures[idx] for idx in valid_indices 
                if idx < len(pressures) and pressures[idx] is not None
            ]
            
            if valid_pressures:
                display_pressures = [p / 10000.0 for p in valid_pressures]
                max_pressure = max(display_pressures)
                avg_pressure = sum(display_pressures) / len(display_pressures)
                
                print(f"   压力: 最大={max_pressure:.2f}, 平均={avg_pressure:.2f} (单位:10^4)")
                
                threshold_display = self.pressure_threshold / 10000.0
                if max_pressure > threshold_display:
                    print(f"   ⚠️  压力超过阈值 ({threshold_display:.2f})!")
                
                print(f"   有效点位数: {len(valid_indices)} (索引: {valid_indices})")
                print("   压力值:")
                for i, idx in enumerate(valid_indices):
                    if idx < len(pressures) and pressures[idx] is not None:
                        display_val = pressures[idx] / 10000.0
                        print(f"     点位[{idx}]: {display_val:6.2f}")
            else:
                print("   压力: 无有效数据")
            
            temperatures = sensor_info.get('temperature', [])
            valid_temps = [
                temperatures[idx] for idx in valid_indices
                if idx < len(temperatures) and temperatures[idx] is not None
            ]
            
            if valid_temps:
                avg_temp = sum(valid_temps) / len(valid_temps)
                print(f"   温度: 平均={avg_temp:.2f}°C")
        
        print("="*60 + "\n")
    
    def detect_fingertip_press(self, pressure_data: Dict[str, Any]) -> Dict[str, bool]:
        """检测指尖是否按压"""
        if pressure_data is None:
            return {'thumb': False, 'index': False, 'middle': False}
        
        press_status = {'thumb': False, 'index': False, 'middle': False}
        threshold_raw = self.pressure_threshold
        
        thumb_sensor = pressure_data.get('sensor_1', {})
        thumb_pressures = thumb_sensor.get('pressure', [])
        valid_thumb = [thumb_pressures[i] for i in [3, 6, 8] 
                       if i < len(thumb_pressures) and thumb_pressures[i] is not None]
        if valid_thumb and max(valid_thumb) > threshold_raw:
            press_status['thumb'] = True
        
        index_sensor = pressure_data.get('sensor_5', {})
        index_pressures = index_sensor.get('pressure', [])
        valid_index = [index_pressures[i] for i in [3, 6, 8]
                       if i < len(index_pressures) and index_pressures[i] is not None]
        if valid_index and max(valid_index) > threshold_raw:
            press_status['index'] = True
        
        middle_sensor = pressure_data.get('sensor_3', {})
        middle_pressures = middle_sensor.get('pressure', [])
        valid_middle = [middle_pressures[i] for i in [3, 6, 8]
                        if i < len(middle_pressures) and middle_pressures[i] is not None]
        if valid_middle and max(valid_middle) > threshold_raw:
            press_status['middle'] = True
        
        return press_status
    
    def continuous_monitor(self, duration: float = 10.0, interval: float = 0.5):
        """连续监控压力数据 - 保留用于全面监控"""
        print(f"\n🔍 开始连续监控 ({duration}秒, 每{interval}秒采样一次)...")
        print("💡 提示: 用手指按压灵巧手指尖,观察压力变化\n")
        
        start_time = time.time()
        sample_count = 0
        
        while time.time() - start_time < duration:
            sample_count += 1
            print(f"\n📍 采样 #{sample_count} (时间: {time.time() - start_time:.1f}s)")
            
            pressure_data = self.read_pressure_once()
            
            if pressure_data:
                press_status = self.detect_fingertip_press(pressure_data)
                
                print("🖐️  指尖按压状态:")
                print(f"   拇指: {'🔴 按下' if press_status['thumb'] else '⚪ 未按'}")
                print(f"   食指: {'🔴 按下' if press_status['index'] else '⚪ 未按'}")
                print(f"   中指: {'🔴 按下' if press_status['middle'] else '⚪ 未按'}")
                
                for finger, sensor_idx in [('拇指', 1), ('中指', 3), ('食指', 5)]:
                    sensor_key = f'sensor_{sensor_idx}'
                    sensor_data = pressure_data.get(sensor_key, {})
                    pressures = sensor_data.get('pressure', [])
                    
                    valid_indices = self.useful_indices.get(sensor_key, [])
                    valid_pressures = [
                        pressures[i] for i in valid_indices
                        if i < len(pressures) and pressures[i] is not None
                    ]
                    
                    if valid_pressures:
                        max_p_raw = max(valid_pressures)
                        max_p_display = max_p_raw / 10000.0
                        print(f"   {finger}最大压力: {max_p_display:.2f} (10^4) [原始值: {max_p_raw:.0f}]")
            else:
                print("⚠️  无法读取压力数据")
            
            time.sleep(interval)
        
        print(f"\n✅ 监控完成 (共采样 {sample_count} 次)")
    
    def test_press_detection_threshold(self):
        """测试压力阈值"""
        print("\n🧪 压力阈值测试")
        print("="*60)
        print("📝 说明: 逐步增加按压力度，找到合适的阈值")
        print("="*60)
        
        print(f"\n当前阈值: {self.pressure_threshold}")
        print("\n请按压指尖，观察压力值变化...")
        print("输入新的阈值数值（或按Enter跳过）:")
        
        try:
            user_input = input().strip()
            if user_input:
                new_threshold = float(user_input)
                self.pressure_threshold = new_threshold
                print(f"✅ 阈值已更新为: {new_threshold}")
        except ValueError:
            print("⚠️  输入无效，保持原阈值")
        
        self.continuous_monitor(duration=5.0, interval=0.5)
    
    def shutdown(self):
        """关闭测试器 - 清除状态"""
        if self.dex3_client:
            if not self.read_only:
                print("\n🔧 停止灵巧手控制...")
                self.dex3_client.stop_control()
                
                # 🆕 清除控制状态
                if STATE_MANAGER_AVAILABLE:
                    robot_state.set_hand_controlling(False, source="pressure_sensor_test")
            
            print("✅ 测试器已关闭")


def main():
    """主测试程序"""
    hand = "left"
    interface = "eth0"
    read_only = False
    
    # 解析命令行参数
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] in ['l', 'left']:
            hand = "left"
        elif sys.argv[i] in ['r', 'right']:
            hand = "right"
        elif sys.argv[i] == '--interface' and i + 1 < len(sys.argv):
            interface = sys.argv[i + 1]
            i += 1
        elif sys.argv[i] == '--read-only':
            read_only = True
        i += 1
    
    print("="*60)
    print("🧪 Dex3 灵巧手压力传感器测试程序")
    print("="*60)
    print(f"🖐️  手: {hand.upper()}")
    print(f"🌐 网络接口: {interface}")
    print(f"📖 模式: {'只读模式（可手动调整）' if read_only else '完整模式（自动控制）'}")
    if STATE_MANAGER_AVAILABLE:
        print("✅ 状态管理器已启用")
    else:
        print("⚠️  状态管理器未启用（独立运行）")
    print("="*60)
    
    tester = PressureSensorTester(hand=hand, interface=interface, read_only=read_only)
    
    try:
        if not tester.initialize():
            sys.exit(1)
        
        print("\n⏳ 等待2秒让传感器稳定...")
        time.sleep(2)
        
        # 测试菜单
        while True:
            print("\n" + "="*60)
            print("📋 测试菜单")
            print("="*60)
            print("1. 🆕 选择特定传感器并监控")
            print("2. 读取选中传感器数据（一次）")
            print("3. 监控选中传感器（10秒）")
            print("4. 监控选中传感器（30秒）")
            print("5. 读取所有传感器数据（详细）")
            print("6. 全面连续监控（10秒）")
            print("7. 测试压力阈值")
            print("8. 指尖按压检测测试（实时）")
            print("q. 退出")
            print("="*60)
            
            choice = input("\n请选择 (1-8/q): ").strip()
            
            if choice == '1':
                if tester.display_sensor_menu():
                    print("\n✅ 传感器选择完成，可以开始监控")
            
            elif choice == '2':
                if not tester.selected_sensors:
                    print("⚠️  请先选择传感器（选项1）")
                else:
                    pressure_data = tester.read_pressure_once()
                    tester.print_selected_sensor_data(pressure_data)
            
            elif choice == '3':
                if not tester.selected_sensors:
                    print("⚠️  请先选择传感器（选项1）")
                else:
                    tester.monitor_selected_sensors(duration=10.0, interval=0.5)
            
            elif choice == '4':
                if not tester.selected_sensors:
                    print("⚠️  请先选择传感器（选项1）")
                else:
                    tester.monitor_selected_sensors(duration=30.0, interval=0.5)
            
            elif choice == '5':
                pressure_data = tester.read_pressure_once()
                tester.read_pressure_data(pressure_data)
            
            elif choice == '6':
                tester.continuous_monitor(duration=10.0, interval=0.5)
            
            elif choice == '7':
                tester.test_press_detection_threshold()
            
            elif choice == '8':
                print("\n🎯 实时指尖按压检测 (按Ctrl+C停止)")
                try:
                    while True:
                        pressure_data = tester.read_pressure_once()
                        if pressure_data:
                            press_status = tester.detect_fingertip_press(pressure_data)
                            
                            status_str = " | ".join([
                                f"拇指: {'🔴' if press_status['thumb'] else '⚪'}",
                                f"食指: {'🔴' if press_status['index'] else '⚪'}",
                                f"中指: {'🔴' if press_status['middle'] else '⚪'}"
                            ])
                            print(f"\r{status_str}", end='', flush=True)
                        
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("\n\n✅ 停止监控")
            
            elif choice.lower() == 'q':
                print("\n👋 退出测试")
                break
            
            else:
                print("⚠️  无效选择，请重试")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  收到中断信号")
    
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        tester.shutdown()


if __name__ == "__main__":
    main()