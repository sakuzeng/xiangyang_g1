#!/usr/bin/env python3
"""
Dex3 灵巧手关节控制器 - 集成压力传感器
"""
import sys
import time
import json
import select
from pathlib import Path
from typing import Optional, List, Dict, Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from common.robot_state_manager import robot_state


class Dex3JointController:
    """灵巧手关节控制器 - 集成压力传感器"""
    
    # 关节映射 - 使用 URDF 精确限位
    JOINT_MAP = {
        'right': [
            {'id': 0, 'name': '拇指-外展/内收', 'range': (-1.0472, 1.0472), 'step': 0.01, 'display_precision': 3},
            {'id': 1, 'name': '拇指-第一指节', 'range': (-1.0472, 0.6109), 'step': 0.01},
            {'id': 2, 'name': '拇指-第二指节', 'range': (-1.7453, 0.0), 'step': 0.01},
            {'id': 3, 'name': '中指-基部', 'range': (0.0, 1.5708), 'step': 0.02},
            {'id': 4, 'name': '中指-指尖', 'range': (0.0, 1.7453), 'step': 0.02},
            {'id': 5, 'name': '食指-基部', 'range': (0.0, 1.5708), 'step': 0.02},
            {'id': 6, 'name': '食指-指尖', 'range': (0.0, 1.7453), 'step': 0.02},
        ],
        'left': [
            {'id': 0, 'name': '拇指-外展/内收', 'range': (-1.0472, 1.0472), 'step': 0.01},
            {'id': 1, 'name': '拇指-第一指节', 'range': (-0.6109, 1.0472), 'step': 0.01},
            {'id': 2, 'name': '拇指-第二指节', 'range': (0.0, 1.7453), 'step': 0.01},
            {'id': 3, 'name': '中指-基部', 'range': (-1.5708, 0.0), 'step': 0.02},
            {'id': 4, 'name': '中指-指尖', 'range': (-1.7453, 0.0), 'step': 0.02},
            {'id': 5, 'name': '食指-基部', 'range': (-1.5708, 0.0), 'step': 0.02},
            {'id': 6, 'name': '食指-指尖', 'range': (-1.7453, 0.0), 'step': 0.02},
        ]
    }
    
    # 🆕 压力传感器映射（精简版）
    PRESSURE_SENSORS = {
        'thumb_tip': {'sensor_id': 1, 'name': '拇指指尖', 'indices': [3, 6, 8]},
        'thumb_base': {'sensor_id': 0, 'name': '拇指基部', 'indices': [0, 2, 9, 11]},
        'index_tip': {'sensor_id': 5, 'name': '食指指尖', 'indices': [3, 6, 8]},
        'index_base': {'sensor_id': 4, 'name': '食指基部', 'indices': [0, 2, 9, 11]},
        'middle_tip': {'sensor_id': 3, 'name': '中指指尖', 'indices': [3, 6, 8]},
        'middle_base': {'sensor_id': 2, 'name': '中指基部', 'indices': [0, 2, 9, 11]},
        'palm_1': {'sensor_id': 6, 'name': '手掌区域1', 'indices': [0, 2, 9, 11]},
        'palm_2': {'sensor_id': 7, 'name': '手掌区域2', 'indices': [0, 2, 9, 11]},
        'palm_3': {'sensor_id': 8, 'name': '手掌区域3', 'indices': [0, 2, 9, 11]},
    }
    
    # 控制参数配置
    DEFAULT_STEP = 0.01
    MIN_STEP = 0.005
    MAX_STEP = 0.05
    STEP_INCREMENT = 0.005
    DISPLAY_PRECISION = 3
    
    # 🆕 压力阈值
    PRESSURE_THRESHOLD = 10.0  # 显示值阈值 (10^4)
    
    def __init__(self, hand: str = "left", interface: str = "eth0"):
        self.hand = hand
        self.interface = interface
        self.dex3 = None
        self.current_positions: List[float] = [0.0] * 7
        self.selected_joint: Optional[int] = None
        self.running = True
        self.emergency_stop = False
        
        # 🆕 压力监控配置
        self.selected_sensors: List[str] = []
        
        self.save_dir = Path("./saved_poses")
        self.save_dir.mkdir(exist_ok=True)
        self.save_file = self.save_dir / f"{hand}_hand_poses.json"
    
    def _clear_stdin_buffer(self):
        """清空键盘输入缓冲区"""
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
    
    def _format_angle(self, rad: float, precision: int = None) -> str:
        """格式化角度显示"""
        precision = precision or self.DISPLAY_PRECISION
        deg = rad * 57.2958
        return f"{rad:{precision+2}.{precision}f} rad ({deg:5.1f}°)"
    
    def initialize(self) -> bool:
        """初始化 - 🆕 使用 _current_jpos_des 跟踪状态"""
        try:
            print(f"🔧 初始化 {self.hand.upper()} 手...")
            ChannelFactoryInitialize(0, self.interface)
            
            self.dex3 = robot_state.get_or_create_hand_client(
                hand=self.hand,
                interface=self.interface
            )
            
            if not self.dex3.initialize_hand():
                print("❌ 初始化失败")
                return False
            
            time.sleep(1)
            
            # 🆕 同步 _current_jpos_des（dex3_client 初始化后已设置）
            # 读取 dex3_client 内部的期望位置
            self.current_positions = self.dex3._current_jpos_des.copy()
            
            print("✅ 初始化成功")
            print(f"📊 {self.hand.upper()} 手当前期望位置 (_current_jpos_des):")
            for i, joint_info in enumerate(self.JOINT_MAP[self.hand]):
                p = self.current_positions[i]
                print(f"   {i}. {joint_info['name']:15s}: {self._format_angle(p)}")
            
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    # ========== 原有的关节控制功能 ==========
    
    def select_joint(self) -> bool:
        """选择关节"""
        print("\n" + "="*80)
        print(f"🖐️  {self.hand.upper()} 手 - 关节选择")
        print("="*80)
        
        joints = self.JOINT_MAP[self.hand]
        
        for joint in joints:
            current = self.current_positions[joint['id']]
            min_val, max_val = joint['range']
            
            print(f"  {joint['id']}. {joint['name']:15s} | "
                  f"当前: {self._format_angle(current):20s} | "
                  f"范围: [{min_val:7.4f}, {max_val:7.4f}] rad")
        
        print("="*80)
        
        choice = input("\n选择关节 (0-6): ").strip()
        
        try:
            idx = int(choice)
            if 0 <= idx <= 6:
                self.selected_joint = idx
                print(f"✅ 已选择: {joints[idx]['name']}")
                return True
            else:
                print("❌ 无效选择")
                return False
        except ValueError:
            print("❌ 输入错误")
            return False
    
    def control_loop(self):
        """控制循环 - 🆕 完全基于 _current_jpos_des"""
        if self.selected_joint is None:
            print("⚠️  未选择关节")
            return
        
        joint_info = self.JOINT_MAP[self.hand][self.selected_joint]
        min_val, max_val = joint_info['range']
        step = joint_info['step']
        
        print("\n" + "="*80)
        print(f"🎮 控制关节: {joint_info['name']}")
        print("="*80)
        print(f"  w - 增加 (+{step:.3f} rad ≈ {step*57.3:.1f}°)")
        print(f"  s - 减少 (-{step:.3f} rad ≈ {step*57.3:.1f}°)")
        print(f"  + - 增大步进 (当前: {step:.3f} rad)")
        print(f"  - - 减小步进 (范围: {self.MIN_STEP:.3f}~{self.MAX_STEP:.3f} rad)")
        print(f"  r - 🆕 同步底层状态")
        print("  ESC - 紧急停止 / q - 返回菜单")
        print("="*80)
        print(f"📊 关节范围: [{min_val:7.4f}, {max_val:7.4f}] rad")
        print("="*80)
        
        import tty
        import termios
        
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            current_step = step
            
            with robot_state.safe_hand_control(hand=self.hand, source="joint_control", timeout=5.0):
                print(f"🔒 已获取控制权 | 状态: {robot_state.get_status_string()}\n")
                
                # 🆕 同步底层 _current_jpos_des
                self.current_positions = self.dex3._current_jpos_des.copy()
                print(f"✅ 当前期望: {self._format_angle(self.current_positions[self.selected_joint])}\n")
                
                while self.running:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        self._clear_stdin_buffer()
                        
                        if key == '\x1b':  # ESC
                            print("\n🚨 紧急停止!")
                            self.emergency_stop = True
                            break
                        elif key == 'q':
                            break
                        elif key == '+':
                            current_step = min(current_step + self.STEP_INCREMENT, self.MAX_STEP)
                            print(f"\r步进: {current_step:.3f} rad ({current_step*57.3:.1f}°)     ", 
                                  end='', flush=True)
                        elif key == '-':
                            current_step = max(current_step - self.STEP_INCREMENT, self.MIN_STEP)
                            print(f"\r步进: {current_step:.3f} rad ({current_step*57.3:.1f}°)     ", 
                                  end='', flush=True)
                        elif key == 'r':
                            # 🆕 同步底层状态
                            print("\n📡 同步底层 _current_jpos_des...")
                            self.current_positions = self.dex3._current_jpos_des.copy()
                            current = self.current_positions[self.selected_joint]
                            print(f"✅ 期望位置: {self._format_angle(current)}")
                        elif key == 'w':
                            # 🆕 基于本地 current_positions 计算
                            target = self.current_positions[self.selected_joint] + current_step
                            target = max(min_val, min(max_val, target))
                            self.current_positions[self.selected_joint] = target
                        
                            # 发送命令（底层会更新 _current_jpos_des）
                            self.dex3.set_joint_positions(self.current_positions, speed_factor=1.0)
                            print(f"\r↑ {self._format_angle(target)} (步进:{current_step:.3f})     ", 
                                  end='', flush=True)
                        elif key == 's':
                            target = self.current_positions[self.selected_joint] - current_step
                            target = max(min_val, min(max_val, target))
                            self.current_positions[self.selected_joint] = target
                        
                            self.dex3.set_joint_positions(self.current_positions, speed_factor=1.0)
                            print(f"\r↓ {self._format_angle(target)} (步进:{current_step:.3f})     ", 
                                  end='', flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n🔓 已释放控制权")
    
    def save_pose(self):
        """💾 保存当前位姿 - 🆕 基于 _current_jpos_des + 添加描述"""
        print("\n" + "="*70)
        print("💾 保存当前位姿")
        print("="*70)
        
        # 🆕 直接使用底层 _current_jpos_des（期望位置）
        print("📡 读取底层期望位置 (_current_jpos_des)...")
        self.current_positions = self.dex3._current_jpos_des.copy()
        
        # 显示当前位置
        print("\n当前期望关节位置:")
        for i, joint_info in enumerate(self.JOINT_MAP[self.hand]):
            print(f"  {i}. {joint_info['name']:15s}: {self._format_angle(self.current_positions[i])}")
        
        print("\n" + "="*70)
        name = input("输入位姿名称 (或q取消): ").strip()
        
        if name.lower() == 'q' or not name:
            print("❌ 已取消")
            return
        
        # 🆕 输入描述信息
        description = input("输入位姿描述 (可选，直接回车跳过): ").strip()
        
        poses = {}
        if self.save_file.exists():
            with open(self.save_file, 'r') as f:
                poses = json.load(f)
        # 生成紧凑格式
        # compact_str = "[" + ", ".join([f"{val:.6f}" for val in self.current_positions]) + "]"
        pose_data = {
            'positions': self.current_positions,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'hand': self.hand
        }
        
        # 🆕 添加描述（如果有）
        if description:
            pose_data['description'] = description
        
        poses[name] = pose_data
        
        with open(self.save_file, 'w') as f:
            json.dump(poses, f, indent=2, ensure_ascii=False)  # 🆕 支持中文
    
        print(f"✅ 位姿 '{name}' 已保存到 {self.save_file}")
        if description:
            print(f"   描述: {description}")
        
        # 显示保存的值
        print("\n📋 已保存的关节值:")
        for i, (joint_info, pos_val) in enumerate(zip(self.JOINT_MAP[self.hand], self.current_positions)):
            print(f"   {i}. {joint_info['name']:15s}: {self._format_angle(pos_val)}")
            
        # ✅ 后处理:将positions数组压缩为单行
        import re
        with open(self.save_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 匹配多行数组并压缩为单行
        def compress_positions(match):
            # 提取所有数字
            numbers = re.findall(r'-?\d+\.\d+', match.group(0))
            return '"positions": [' + ', '.join(numbers) + ']'
        
        content = re.sub(
            r'"positions":\s*\[\s*([\s\S]*?)\s*\]',
            compress_positions,
            content
        )
        
        with open(self.save_file, 'w', encoding='utf-8') as f:
            f.write(content)
    def load_pose(self):
        """加载位姿 - 🆕 显示描述"""
        if not self.save_file.exists():
            print("⚠️  无保存位姿")
            return
        
        with open(self.save_file, 'r') as f:
            poses = json.load(f)
        
        if not poses:
            print("⚠️  无保存位姿")
            return
        
        print("\n📂 保存的位姿:")
        for i, (name, data) in enumerate(poses.items(), 1):
            timestamp = data.get('timestamp', 'N/A')
            description = data.get('description', '')
            desc_text = f" - {description}" if description else ""
            print(f"  {i}. {name} ({timestamp}){desc_text}")
        
        choice = input("\n选择 (或q取消): ").strip()

        if choice.lower() == 'q':
            return
        
        try:
            idx = int(choice) - 1
            pose_name = list(poses.keys())[idx]
            pose_data = poses[pose_name]
            positions = pose_data['positions']
            
            print(f"📥 加载: {pose_name}")
            if 'description' in pose_data:
                print(f"   描述: {pose_data['description']}")
            
            with robot_state.safe_hand_control(hand=self.hand, source="load_pose", timeout=10.0):
                self.dex3.set_joint_positions(positions, speed_factor=1.0)
                self.current_positions = positions
                print("✅ 加载完成")
                
                # 显示加载的位置
                print("\n📋 已加载的关节值:")
                for i, (joint_info, pos_val) in enumerate(zip(self.JOINT_MAP[self.hand], positions)):
                    print(f"   {i}. {joint_info['name']:15s}: {self._format_angle(pos_val)}")
        except (ValueError, IndexError):
            print("❌ 无效选择")
        except RuntimeError as e:
            print(f"❌ {e}")
        except Exception as e:
            print(f"❌ 加载失败: {e}")
    
    def show_current_pose(self):
        """显示当前位姿"""
        print(f"\n📊 {self.hand.upper()} 手当前位置:")
        print("="*80)
        for joint in self.JOINT_MAP[self.hand]:
            pos = self.current_positions[joint['id']]
            min_val, max_val = joint['range']
            
            range_size = max_val - min_val
            percentage = ((pos - min_val) / range_size) * 100 if range_size > 0 else 0
            
            print(f"  {joint['name']:15s}: {self._format_angle(pos):20s} "
                  f"({percentage:5.1f}% 范围)")
        print("="*80)
    
    # ========== 🆕 压力传感器功能 ==========
    
    def select_pressure_sensors(self) -> bool:
        """选择要监控的压力传感器"""
        print("\n" + "="*70)
        print(f"🔍 {self.hand.upper()} 手 - 压力传感器选择")
        print("="*70)
        
        sensor_list = list(self.PRESSURE_SENSORS.items())
        
        for i, (key, info) in enumerate(sensor_list, 1):
            print(f"  {i}. {info['name']:12s} (sensor_{info['sensor_id']}, {len(info['indices'])}点)")
        
        print("\n" + "="*70)
        print("💡 选择方式:")
        print("  - 单个: 输入数字 (如: 1)")
        print("  - 多个: 用逗号分隔 (如: 1,3,5)")
        print("  - 全部: 输入 'all'")
        print("  - 取消: 输入 'q'")
        print("="*70)
        
        choice = input("\n请选择: ").strip()
        
        if choice.lower() == 'q':
            return False
        
        self.selected_sensors = []
        
        if choice.lower() == 'all':
            self.selected_sensors = [key for key, _ in sensor_list]
            print(f"✅ 已选择全部 {len(self.selected_sensors)} 个传感器")
        else:
            try:
                selected_indices = [int(x.strip()) for x in choice.split(',')]
                for idx in selected_indices:
                    if 1 <= idx <= len(sensor_list):
                        key, _ = sensor_list[idx - 1]
                        self.selected_sensors.append(key)
                    else:
                        print(f"⚠️  无效选项: {idx}")
                
                if self.selected_sensors:
                    print(f"✅ 已选择 {len(self.selected_sensors)} 个传感器:")
                    for key in self.selected_sensors:
                        print(f"  - {self.PRESSURE_SENSORS[key]['name']}")
                    return True
                else:
                    print("❌ 未选择任何传感器")
                    return False
            except ValueError:
                print("❌ 输入格式错误")
                return False
        
        return True
    
    def read_pressure_data(self) -> Optional[Dict[str, Any]]:
        """读取压力数据"""
        if self.dex3 is None:
            print("❌ 灵巧手未初始化")
            return None
        
        return self.dex3.get_pressure_data(timeout=1.0)
    
    def show_selected_pressure(self):
        """显示选中传感器的压力数据"""
        if not self.selected_sensors:
            print("⚠️  未选择传感器,请先执行选项5")
            return
        
        pressure_data = self.read_pressure_data()
        
        if not pressure_data:
            print("❌ 无法读取压力数据")
            return
        
        print("\n" + "="*70)
        print(f"📊 {self.hand.upper()} 手压力传感器数据")
        print("="*70)
        
        for sensor_key in self.selected_sensors:
            sensor_info = self.PRESSURE_SENSORS[sensor_key]
            sensor_data_key = f"sensor_{sensor_info['sensor_id']}"
            sensor_data = pressure_data.get(sensor_data_key, {})
            
            print(f"\n🔹 {sensor_info['name']} ({sensor_data_key}):")
            
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
                
                status = '🔴按下' if max_pressure > self.PRESSURE_THRESHOLD else '⚪未按'
                
                print(f"   压力: 最大={max_pressure:6.2f}, 平均={avg_pressure:6.2f} (10^4) {status}")
                print(f"   点位值: ", end='')
                for i, idx in enumerate(valid_indices):
                    if idx < len(pressures) and pressures[idx] is not None:
                        print(f"[{idx}]={pressures[idx]/10000.0:5.2f} ", end='')
                print()
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
        
        print("="*70)
    
    def monitor_pressure_realtime(self, duration: float = 10.0):
        """实时监控压力（精简版）"""
        if not self.selected_sensors:
            print("⚠️  未选择传感器,请先执行选项5")
            return
        
        print(f"\n🔍 实时压力监控 ({duration}秒, 按Ctrl+C停止)")
        print(f"💡 监控传感器: {', '.join([self.PRESSURE_SENSORS[k]['name'] for k in self.selected_sensors])}\n")
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                pressure_data = self.read_pressure_data()
                
                if pressure_data:
                    status_parts = []
                    
                    for sensor_key in self.selected_sensors:
                        sensor_info = self.PRESSURE_SENSORS[sensor_key]
                        sensor_data_key = f"sensor_{sensor_info['sensor_id']}"
                        sensor_data = pressure_data.get(sensor_data_key, {})
                        pressures = sensor_data.get('pressure', [])
                        
                        valid_pressures = [
                            pressures[idx] for idx in sensor_info['indices']
                            if idx < len(pressures) and pressures[idx] is not None
                        ]
                        
                        if valid_pressures:
                            max_p = max(valid_pressures) / 10000.0
                            status = '🔴' if max_p > self.PRESSURE_THRESHOLD else '⚪'
                            status_parts.append(f"{sensor_info['name']}:{status}{max_p:5.2f}")
                    
                    status_line = "\r" + " | ".join(status_parts) + " (10^4)"
                    print(status_line, end='', flush=True)
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n✅ 停止监控")
        
        print("\n")
    
    # ========== 主程序 ==========
    
    def shutdown(self):
        """关闭"""
        if self.dex3:
            if self.emergency_stop:
                print(f"🔧 紧急停止 {self.hand.upper()} 手...")
                robot_state.emergency_stop_hand(self.hand)
            else:
                print("🔧 正常关闭...")
                self.dex3.stop_control()
            
            robot_state.reset_hand_state(self.hand)
            print("✅ 已关闭")


def main():
    """主程序"""
    hand = "left"
    interface = "eth0"
    
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg in ['l', 'left']:
            hand = "left"
        elif arg in ['r', 'right']:
            hand = "right"
        elif arg == '--interface' and i + 1 < len(sys.argv):
            interface = sys.argv[i + 1]
    
    print("="*80)
    print("🎮 Dex3 灵巧手控制器 (集成压力传感器)")
    print("="*80)
    print(f"🖐️  手: {hand.upper()}")
    print(f"🌐 接口: {interface}")
    print(f"🛡️  限位保护: 使用 URDF 精确限位")
    print(f"📊 压力阈值: {Dex3JointController.PRESSURE_THRESHOLD:.2f} (10^4)")
    print("="*80)
    
    controller = Dex3JointController(hand=hand, interface=interface)
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        while controller.running:
            print("\n" + "="*80)
            print("📋 主菜单")
            print("="*80)
            print("1. 选择关节并控制")
            print("2. 💾 保存当前位姿")  # 🆕 独立选项
            print("3. 加载保存的位姿")
            print("4. 查看当前位置")
            print("5. 🆕 选择压力传感器")
            print("6. 🆕 查看选中传感器数据")
            print("7. 🆕 实时监控 (10秒)")
            print("8. 🆕 实时监控 (30秒)")
            print("s. 显示状态")
            print("q. 退出")
            print("="*80)
            
            choice = input("\n选择: ").strip()
            
            if choice == '1':
                if controller.select_joint():
                    controller.control_loop()
            elif choice == '2':  # 🆕 保存位姿独立
                controller.save_pose()
            elif choice == '3':
                controller.load_pose()
            elif choice == '4':
                controller.show_current_pose()
            elif choice == '5':
                controller.select_pressure_sensors()
            elif choice == '6':
                controller.show_selected_pressure()
            elif choice == '7':
                controller.monitor_pressure_realtime(duration=10.0)
            elif choice == '8':
                controller.monitor_pressure_realtime(duration=30.0)
            elif choice.lower() == 's':
                print(f"\n📊 系统状态:")
                print(f"   {robot_state.get_status_string()}")
                print(f"   当前手: {robot_state.get_hand_status(controller.hand)}")
                if controller.selected_sensors:
                    print(f"   监控传感器: {len(controller.selected_sensors)}个")
            elif choice.lower() == 'q':
                controller.running = False
                break
            else:
                print("⚠️  无效选择")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  收到中断信号")
        controller.emergency_stop = True
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()