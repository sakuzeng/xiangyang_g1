#!/usr/bin/env python3
"""
G1 手臂关节控制器 - 优化版
特性:
- 使用与 arm_client 一致的 URDF 限位
- 基于底层控制周期优化步进值
- 统一精度显示
- 🆕 保存位姿独立菜单
- 🆕 FK计算末端Torso坐标 (用于界限采集)
"""
import sys
import time
import json
import select
from pathlib import Path
from typing import Optional, List, Tuple

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.arm.arm_client import JointIndex

import os
from pathlib import Path
# 添加项目根目录到路径 (为了导入 xiangyang 包)
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from xiangyang.loco.common.robot_state_manager import robot_state
from xiangyang.loco.phone.screen_to_ik import ScreenToIKSolver


class ArmJointController:
    """手臂关节控制器 - 优化版"""
    
    # 关节映射 - 🆕 使用与 arm_client 一致的 URDF 限位
    JOINT_MAP = {
        'left': [
            {
                'id': 0, 
                'name': '左肩前后摆动动作', 
                'range': (-3.0892, 2.6704),  # -177°~153°
                'step': 0.02,
                'display_precision': 3
            },
            {
                'id': 1, 
                'name': '左肩左右摆动动作', 
                'range': (-1.5882, 2.2515),  # -91°~129°
                'step': 0.02
            },
            {
                'id': 2, 
                'name': '左大臂自旋动作', 
                'range': (-2.618, 2.618),  # ±150°
                'step': 0.02
            },
            {
                'id': 3, 
                'name': '左肘前后摆动动作', 
                'range': (-1.0472, 2.0944),  # -60°~120°
                'step': 0.02
            },
            {
                'id': 4, 
                'name': '左手腕旋转动作', 
                'range': (-1.9722, 1.9722),  # ±113°
                'step': 0.02
            },
            {
                'id': 5, 
                'name': '左手腕前后摆动动作', 
                'range': (-1.6144, 1.6144),  # ±92.5°
                'step': 0.02
            },
            {
                'id': 6, 
                'name': '左手腕左右摆动动作', 
                'range': (-1.6144, 1.6144),  # ±92.5°
                'step': 0.02
            },
        ],
        'right': [
            {
                'id': 0, 
                'name': '右肩前后摆动动作', 
                'range': (-3.0892, 2.6704),  # -177°~153°
                'step': 0.02
            },
            {
                'id': 1, 
                'name': '右肩左右摆动动作', 
                'range': (-2.2515, 1.5882),  # -129°~91° (镜像对称)
                'step': 0.02
            },
            {
                'id': 2, 
                'name': '右大臂自旋动作', 
                'range': (-2.618, 2.618),  # ±150°
                'step': 0.02
            },
            {
                'id': 3, 
                'name': '右肘前后摆动动作', 
                'range': (-1.0472, 2.0944),  # -60°~120°
                'step': 0.02
            },
            {
                'id': 4, 
                'name': '右手腕旋转动作', 
                'range': (-1.9722, 1.9722),  # ±113°
                'step': 0.02
            },
            {
                'id': 5, 
                'name': '右手腕前后摆动动作', 
                'range': (-1.6144, 1.6144),  # ±92.5°
                'step': 0.02
            },
            {
                'id': 6, 
                'name': '右手腕左右摆动动作', 
                'range': (-1.6144, 1.6144),  # ±92.5°
                'step': 0.02
            },
        ]
    }
    
    # 控制参数配置
    DEFAULT_STEP = 0.02
    MIN_STEP = 0.005
    MAX_STEP = 0.1
    STEP_INCREMENT = 0.005
    DISPLAY_PRECISION = 3
    
    def __init__(self, arm: str = "left", interface: str = "eth0"):
        self.arm = arm
        self.interface = interface
        self.arm_client = None
        self.current_positions: List[float] = [0.0] * 14
        self.selected_joint: Optional[int] = None
        self.running = True
        self.emergency_stop = False
        
        self.save_dir = Path("./saved_poses")
        self.save_dir.mkdir(exist_ok=True)
        self.save_file = self.save_dir / f"{arm}_arm_poses.json"
        
        # 🆕 FK求解器 (仅支持左臂)
        self.ik_solver = None
        if arm == "left":
            try:
                self.ik_solver = ScreenToIKSolver()
                print("✅ FK求解器初始化成功")
            except Exception as e:
                print(f"⚠️  FK求解器初始化失败: {e}")
    
    def _get_arm_offset(self) -> int:
        """获取手臂在14DOF数组中的偏移量"""
        return 0 if self.arm == 'left' else 7
    
    def _clear_stdin_buffer(self):
        """清空键盘输入缓冲区"""
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
    
    def _format_angle(self, rad: float, precision: int = None) -> str:
        """格式化角度显示"""
        precision = precision or self.DISPLAY_PRECISION
        deg = rad * 57.2958
        return f"{rad:{precision+2}.{precision}f} rad ({deg:5.1f}°)"
    
    def _get_current_end_position(self) -> Optional[Tuple[float, float, float]]:
        """
        🆕 通过FK计算当前末端位置(x, y, z) - 基于 _current_jpos_des
        
        Returns:
            Tuple[x, y, z]: Torso坐标系下的末端位置(米), 失败返回None
        """
        if self.ik_solver is None:
            print("⚠️  FK求解器未初始化 (仅支持左臂)")
            return None
        
        try:
            offset = self._get_arm_offset()
            
            # 获取当前关节角度 (索引0-6是左臂)
            current_joints = self.current_positions[offset:offset+7]
            
            # 构造完整状态向量 [0.0, j1, j2, ..., j7, 0.0]
            full_state = [0.0] + list(current_joints) + [0.0]
            
            # FK计算
            current_frame = self.ik_solver.chain.forward_kinematics(full_state)
            
            # 提取位置 (4x4变换矩阵的最后一列前三个元素)
            x = current_frame[0, 3]
            y = current_frame[1, 3]
            z = current_frame[2, 3]
            
            return (x, y, z)
            
        except Exception as e:
            print(f"⚠️  FK计算失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def show_end_effector_position(self):
        """
        🆕 显示末端执行器Torso坐标
        """
        if self.ik_solver is None:
            print("\n⚠️  FK功能仅支持左臂")
            input("按回车继续...")
            return
        
        print("\n" + "="*80)
        print("📍 末端执行器位置 (Torso坐标系)")
        print("="*80)
        
        # 同步底层状态
        self.current_positions = self.arm_client._current_jpos_des.copy()
        
        pos = self._get_current_end_position()
        
        if pos is None:
            print("❌ 无法计算末端位置")
            input("按回车继续...")
            return
        
        x, y, z = pos
        
        print(f"\n当前末端位置:")
        print(f"   X = {x:+.4f} m  ({x*1000:+7.1f} mm)")
        print(f"   Y = {y:+.4f} m  ({y*1000:+7.1f} mm)")
        print(f"   Z = {z:+.4f} m  ({z*1000:+7.1f} mm)")
        
        # 🆕 距离原点的欧氏距离
        distance = (x**2 + y**2 + z**2)**0.5
        print(f"\n距离Torso原点: {distance:.4f} m ({distance*1000:.1f} mm)")
        
        # 🆕 用于采集界限数据的快捷输出
        print("\n" + "-"*80)
        print("📋 复制用数据 (方便记录到Excel/文档):")
        print("-"*80)
        print(f"X={x:.4f}, Y={y:.4f}, Z={z:.4f}")
        print(f"{x:.4f}\t{y:.4f}\t{z:.4f}")  # Tab分隔 (Excel友好)
        
        # 🆕 显示当前关节角度 (用于复现)
        offset = self._get_arm_offset()
        print("\n当前关节角度:")
        for i, joint_info in enumerate(self.JOINT_MAP[self.arm]):
            angle = self.current_positions[offset + i]
            print(f"   [{i}] {joint_info['name']:12s}: {self._format_angle(angle)}")
        
        print("="*80)
        input("按回车继续...")
    
    def initialize(self) -> bool:
        """初始化 - 🆕 使用 _current_jpos_des 跟踪状态"""
        try:
            print(f"🔧 初始化 {self.arm.upper()} 手臂...")
            ChannelFactoryInitialize(0, self.interface)
            
            self.arm_client = robot_state.get_or_create_arm_client(self.interface)
            
            if not self.arm_client.initialize_arms():
                print("❌ 初始化失败")
                return False
            
            time.sleep(2)
            
            # 🆕 同步 _current_jpos_des（arm_client 初始化后已设置）
            self.current_positions = self.arm_client._current_jpos_des.copy()
            
            offset = self._get_arm_offset()
            
            print("✅ 初始化成功")
            print(f"📊 {self.arm.upper()} 手臂当前期望位置 (_current_jpos_des):")
            for i, joint_info in enumerate(self.JOINT_MAP[self.arm]):
                p = self.current_positions[offset + i]
                print(f"   {i}. {joint_info['name']:12s}: {self._format_angle(p)}")
            
            # 🆕 显示初始末端位置
            if self.ik_solver:
                pos = self._get_current_end_position()
                if pos:
                    print(f"\n📍 初始末端位置: X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}")
            
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def select_joint(self) -> bool:
        """选择关节"""
        print("\n" + "="*80)
        print(f"💪 {self.arm.upper()} 手臂 - 关节选择")
        print("="*80)
        
        offset = self._get_arm_offset()
        joints = self.JOINT_MAP[self.arm]
        
        for joint in joints:
            current = self.current_positions[offset + joint['id']]
            min_val, max_val = joint['range']
            
            print(f"  {joint['id']}. {joint['name']:12s} | "
                  f"当前: {self._format_angle(current):20s} | "
                  f"范围: [{min_val:6.3f}, {max_val:6.3f}] rad")
        
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
        """控制循环 - 🆕 实时显示FK坐标"""
        if self.selected_joint is None:
            print("⚠️  未选择关节")
            return
        
        joint_info = self.JOINT_MAP[self.arm][self.selected_joint]
        min_val, max_val = joint_info['range']
        step = joint_info['step']
        offset = self._get_arm_offset()
        
        print("\n" + "="*80)
        print(f"🎮 控制关节: {joint_info['name']}")
        print("="*80)
        print(f"  w - 增加 (+{step:.3f} rad ≈ {step*57.3:.1f}°)")
        print(f"  s - 减少 (-{step:.3f} rad ≈ {step*57.3:.1f}°)")
        print(f"  + - 增大步进 (当前: {step:.3f} rad)")
        print(f"  - - 减小步进 (范围: {self.MIN_STEP:.3f}~{self.MAX_STEP:.3f} rad)")
        print(f"  r - 同步底层状态")
        print(f"  p - 🆕 显示末端Torso坐标")
        print("  ESC - 紧急停止 / q - 返回菜单")
        print("="*80)
        print(f"📊 关节范围: [{min_val:6.3f}, {max_val:6.3f}] rad "
              f"({min_val*57.3:6.1f}° ~ {max_val*57.3:6.1f}°)")
        print("="*80)
        
        import tty
        import termios
        
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            current_step = step
            
            with robot_state.safe_arm_control(arm=self.arm, source="joint_control", timeout=5.0):
                print(f"🔒 已获取控制权 | 状态: {robot_state.get_status_string()}\n")
                
                # 同步底层状态
                self.current_positions = self.arm_client._current_jpos_des.copy()
                print(f"✅ 当前期望: {self._format_angle(self.current_positions[offset + self.selected_joint])}\n")
                
                # 🆕 显示初始FK坐标
                if self.ik_solver:
                    pos = self._get_current_end_position()
                    if pos:
                        print(f"📍 初始位置: X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}\n")
                
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
                            # 同步底层状态
                            print("\n📡 同步底层 _current_jpos_des...")
                            self.current_positions = self.arm_client._current_jpos_des.copy()
                            current = self.current_positions[offset + self.selected_joint]
                            print(f"✅ 期望位置: {self._format_angle(current)}")
                            
                            # 🆕 同时显示FK坐标
                            if self.ik_solver:
                                pos = self._get_current_end_position()
                                if pos:
                                    print(f"📍 末端位置: X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}")
                        elif key == 'p':
                            # 🆕 显示详细FK坐标
                            print("\n" + "-"*80)
                            if self.ik_solver:
                                pos = self._get_current_end_position()
                                if pos:
                                    print(f"📍 末端Torso坐标:")
                                    print(f"   X = {pos[0]:+.4f} m")
                                    print(f"   Y = {pos[1]:+.4f} m")
                                    print(f"   Z = {pos[2]:+.4f} m")
                                    print(f"\n复制用: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
                            else:
                                print("⚠️  FK功能仅支持左臂")
                            print("-"*80)
                        elif key == 'w':
                            target = self.current_positions[offset + self.selected_joint] + current_step
                            target = max(min_val, min(max_val, target))
                            self.current_positions[offset + self.selected_joint] = target
                        
                            self.arm_client.set_joint_positions(
                                self.current_positions,
                                speed_factor=1.0
                            )
                            
                            # 🆕 同时显示FK坐标
                            if self.ik_solver:
                                pos = self._get_current_end_position()
                                if pos:
                                    print(f"\r↑ {self._format_angle(target)} | FK: X={pos[0]:+.3f} Y={pos[1]:+.3f} Z={pos[2]:+.3f}     ", 
                                          end='', flush=True)
                                else:
                                    print(f"\r↑ {self._format_angle(target)} (步进:{current_step:.3f})     ", 
                                          end='', flush=True)
                            else:
                                print(f"\r↑ {self._format_angle(target)} (步进:{current_step:.3f})     ", 
                                      end='', flush=True)
                        elif key == 's':
                            target = self.current_positions[offset + self.selected_joint] - current_step
                            target = max(min_val, min(max_val, target))
                            self.current_positions[offset + self.selected_joint] = target
                        
                            self.arm_client.set_joint_positions(
                                self.current_positions,
                                speed_factor=1.0
                            )
                            
                            # 🆕 同时显示FK坐标
                            if self.ik_solver:
                                pos = self._get_current_end_position()
                                if pos:
                                    print(f"\r↓ {self._format_angle(target)} | FK: X={pos[0]:+.3f} Y={pos[1]:+.3f} Z={pos[2]:+.3f}     ", 
                                          end='', flush=True)
                                else:
                                    print(f"\r↓ {self._format_angle(target)} (步进:{current_step:.3f})     ", 
                                          end='', flush=True)
                            else:
                                print(f"\r↓ {self._format_angle(target)} (步进:{current_step:.3f})     ", 
                                      end='', flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n🔓 已释放控制权")

    def save_pose(self):
        """💾 保存当前位姿"""
        print("\n" + "="*70)
        print("💾 保存当前位姿")
        print("="*70)
        
        offset = self._get_arm_offset()
        
        print("📡 读取底层期望位置 (_current_jpos_des)...")
        self.current_positions = self.arm_client._current_jpos_des.copy()
        
        # 显示当前位置
        print("\n当前期望关节位置:")
        for i, joint_info in enumerate(self.JOINT_MAP[self.arm]):
            p = self.current_positions[offset + i]
            print(f"  {i}. {joint_info['name']:12s}: {self._format_angle(p)}")
        
        # 🆕 显示FK坐标
        if self.ik_solver:
            pos = self._get_current_end_position()
            if pos:
                print(f"\n📍 末端Torso坐标: X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}")
        
        print("\n" + "="*70)
        name = input("输入位姿名称 (或q取消): ").strip()
        
        if name.lower() == 'q' or not name:
            print("❌ 已取消")
            return
        
        description = input("输入位姿描述 (可选，直接回车跳过): ").strip()
        
        # 加载已有位姿
        poses = {}
        if self.save_file.exists():
            with open(self.save_file, 'r') as f:
                poses = json.load(f)
        
        arm_positions = self.current_positions[offset:offset+7]
        pose_data = {
            'positions': arm_positions,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'arm': self.arm
        }
        
        if description:
            pose_data['description'] = description
        
        # 🆕 保存FK坐标
        if self.ik_solver:
            pos = self._get_current_end_position()
            if pos:
                pose_data['torso_coord'] = {
                    'x': float(pos[0]),
                    'y': float(pos[1]),
                    'z': float(pos[2])
                }
        
        poses[name] = pose_data
        
        with open(self.save_file, 'w') as f:
            json.dump(poses, f, indent=2, ensure_ascii=False)
    
        print(f"✅ 位姿 '{name}' 已保存到 {self.save_file}")
        if description:
            print(f"   描述: {description}")
        
        # 显示保存的值
        print("\n📋 已保存的关节值:")
        for i, (joint_info, pos_val) in enumerate(zip(self.JOINT_MAP[self.arm], arm_positions)):
            print(f"   {i}. {joint_info['name']:12s}: {self._format_angle(pos_val)}")
        
        # 压缩positions数组为单行
        import re
        with open(self.save_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        def compress_positions(match):
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
        """加载位姿"""
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
            
            # 🆕 显示保存的Torso坐标
            if 'torso_coord' in data:
                coord = data['torso_coord']
                coord_text = f" [X={coord['x']:.3f}, Y={coord['y']:.3f}, Z={coord['z']:.3f}]"
            else:
                coord_text = ""
            
            print(f"  {i}. {name} ({timestamp}){desc_text}{coord_text}")
        
        choice = input("\n选择 (或q取消): ").strip()
        if choice.lower() == 'q':
            return
        
        try:
            idx = int(choice) - 1
            pose_name = list(poses.keys())[idx]
            pose_data = poses[pose_name]
            saved_positions = pose_data['positions']
            
            print(f"📥 加载: {pose_name}")
            if 'description' in pose_data:
                print(f"   描述: {pose_data['description']}")
            if 'torso_coord' in pose_data:
                coord = pose_data['torso_coord']
                print(f"   保存的Torso坐标: X={coord['x']:.4f}, Y={coord['y']:.4f}, Z={coord['z']:.4f}")
            
            offset = self._get_arm_offset()
            
            target_positions = self.arm_client._current_jpos_des.copy()
            target_positions[offset:offset+7] = saved_positions
            
            with robot_state.safe_arm_control(arm=self.arm, source="load_pose", timeout=10.0):
                self.arm_client.set_joint_positions(target_positions, speed_factor=1.0)
                self.current_positions = self.arm_client._current_jpos_des.copy()
                print("✅ 加载完成")
                
                # 显示加载的位置
                print("\n📋 已加载的关节值:")
                for i, (joint_info, pos_val) in enumerate(zip(self.JOINT_MAP[self.arm], saved_positions)):
                    print(f"   {i}. {joint_info['name']:12s}: {self._format_angle(pos_val)}")
                
                # 🆕 验证FK坐标
                if self.ik_solver:
                    pos = self._get_current_end_position()
                    if pos and 'torso_coord' in pose_data:
                        coord = pose_data['torso_coord']
                        error = ((pos[0]-coord['x'])**2 + (pos[1]-coord['y'])**2 + (pos[2]-coord['z'])**2)**0.5
                        print(f"\n📍 实际末端位置: X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}")
                        print(f"   与保存值误差: {error*1000:.2f} mm")
        except (ValueError, IndexError):
            print("❌ 无效选择")
        except RuntimeError as e:
            print(f"❌ {e}")
        except Exception as e:
            print(f"❌ 加载失败: {e}")

    def show_current_pose(self):
        """显示当前位姿"""
        offset = self._get_arm_offset()
        
        self.current_positions = self.arm_client._current_jpos_des.copy()
        
        print(f"\n📊 {self.arm.upper()} 手臂当前期望位置 (_current_jpos_des):")
        print("="*80)
        for joint in self.JOINT_MAP[self.arm]:
            pos = self.current_positions[offset + joint['id']]
            min_val, max_val = joint['range']
            
            range_size = max_val - min_val
            percentage = ((pos - min_val) / range_size) * 100 if range_size > 0 else 0
            
            print(f"  {joint['name']:12s}: {self._format_angle(pos):20s} "
                  f"({percentage:5.1f}% 范围)")
        
        # 🆕 显示FK坐标
        if self.ik_solver:
            pos = self._get_current_end_position()
            if pos:
                print(f"\n📍 末端Torso坐标:")
                print(f"   X = {pos[0]:+.4f} m  ({pos[0]*1000:+7.1f} mm)")
                print(f"   Y = {pos[1]:+.4f} m  ({pos[1]*1000:+7.1f} mm)")
                print(f"   Z = {pos[2]:+.4f} m  ({pos[2]*1000:+7.1f} mm)")
        
        print("="*80)
    
    def shutdown(self):
        """关闭"""
        if self.arm_client:
            if self.emergency_stop:
                print(f"🔧 紧急停止 {self.arm.upper()} 手臂...")
                robot_state.emergency_stop_arm(self.arm)
            else:
                print("🔧 正常关闭...")
                self.arm_client.stop_control()
            
            robot_state.reset_arm_state(self.arm)
            print("✅ 已关闭")


def main():
    """主程序"""
    arm = "left"
    interface = "eth0"
    
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg in ['l', 'left']:
            arm = "left"
        elif arg in ['r', 'right']:
            arm = "right"
        elif arg == '--interface' and i + 1 < len(sys.argv):
            interface = sys.argv[i + 1]
    
    print("="*80)
    print("🎮 G1 手臂关节控制器 (优化版 + FK功能)")
    print("="*80)
    print(f"💪 手臂: {arm.upper()}")
    print(f"🌐 接口: {interface}")
    print(f"🛡️  限位保护: 使用 URDF 精确限位")
    print(f"📊 默认步进: {ArmJointController.DEFAULT_STEP:.3f} rad "
          f"({ArmJointController.DEFAULT_STEP*57.3:.1f}°)")
    if arm == "left":
        print(f"📍 FK功能: 已启用 (可查看末端Torso坐标)")
    else:
        print(f"⚠️  FK功能: 仅支持左臂")
    print("="*80)
    
    controller = ArmJointController(arm=arm, interface=interface)
    
    try:
        if not controller.initialize():
            sys.exit(1)
        
        while controller.running:
            print("\n" + "="*80)
            print("📋 主菜单")
            print("="*80)
            print("1. 选择关节并控制")
            print("2. 💾 保存当前位姿")
            print("3. 加载保存的位姿")
            print("4. 查看当前位置")
            if controller.ik_solver:
                print("5. 📍 查看末端Torso坐标 (FK)")  # 🆕
            print("s. 显示状态")
            print("q. 退出")
            print("="*80)
            
            choice = input("\n选择: ").strip()
            
            if choice == '1':
                if controller.select_joint():
                    controller.control_loop()
            elif choice == '2':
                controller.save_pose()
            elif choice == '3':
                controller.load_pose()
            elif choice == '4':
                controller.show_current_pose()
            elif choice == '5' and controller.ik_solver:  # 🆕
                controller.show_end_effector_position()
            elif choice.lower() == 's':
                print(f"\n📊 系统状态:")
                print(f"   {robot_state.get_status_string()}")
                print(f"   当前手臂: {robot_state.get_arm_status(controller.arm)}")
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
