#!/usr/bin/env python3
"""
G1机器人手臂 - 基于预设姿态的Z轴移动控制
功能: 从JSON文件读取姿态,计算Z轴移动后的新关节角度
"""
import sys
import json
import ikpy.chain
import ikpy.link
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
from datetime import datetime
from typing import Tuple, Dict

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))


# ================= 1. URDF解析 (复用) =================
def get_chain_from_urdf(urdf_file, base_link_name, tip_link_name):
    """从URDF构建运动学链"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    link_parent_joint = {}
    joints = {}
    
    for joint in root.findall('joint'):
        name = joint.get('name')
        child = joint.find('child').get('link')
        parent = joint.find('parent').get('link')
        joint_type = joint.get('type', 'fixed')
        origin = joint.find('origin')
        if origin is not None:
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
        else:
            xyz, rpy = [0, 0, 0], [0, 0, 0]
        axis_elem = joint.find('axis')
        axis = [float(x) for x in axis_elem.get('xyz').split()] if axis_elem is not None else [0, 0, 0]
        limit = joint.find('limit')
        if limit is not None:
            lower = float(limit.get('lower', -3.14))
            upper = float(limit.get('upper', 3.14))
        else:
            lower, upper = -np.inf, np.inf

        joints[name] = {
            'type': joint_type, 'xyz': xyz, 'rpy': rpy, 
            'axis': axis, 'bounds': (lower, upper),
            'parent_link': parent, 'child_link': child
        }
        link_parent_joint[child] = name

    chain_joints = []
    current_link = tip_link_name
    while current_link != base_link_name:
        if current_link not in link_parent_joint:
            raise ValueError(f"断链! 无法从 {tip_link_name} 回溯到 {base_link_name}")
        joint_name = link_parent_joint[current_link]
        joint_data = joints[joint_name]
        chain_joints.insert(0, (joint_name, joint_data))
        current_link = joint_data['parent_link']

    ikpy_links = []
    ikpy_links.append(ikpy.link.OriginLink()) 
    active_mask = [False]

    for name, data in chain_joints:
        is_fixed = (data['type'] == 'fixed')
        if is_fixed:
            j_type = 'fixed'
            ik_rotation = None
            active_mask.append(False)
        else:
            j_type = 'revolute'
            ik_rotation = data['axis']
            active_mask.append(True)

        link = ikpy.link.URDFLink(
            name=name,
            origin_translation=data['xyz'],
            origin_orientation=data['rpy'],
            rotation=ik_rotation,
            bounds=data['bounds'],
            joint_type=j_type
        )
        ikpy_links.append(link)

    return ikpy.chain.Chain(ikpy_links, name=f"{base_link_name}_to_{tip_link_name}", active_links_mask=active_mask)


# ================= 2. 姿态加载器 =================
class PoseLoader:
    """从JSON文件加载预设姿态"""
    
    def __init__(self, pose_file: str = "../arm_control/saved_poses/left_arm_poses.json"):
        self.pose_file = Path(pose_file)
        self.poses = {}
        self.load_poses()
    
    def load_poses(self):
        """加载姿态文件"""
        if not self.pose_file.exists():
            raise FileNotFoundError(f"姿态文件不存在: {self.pose_file}")
        
        with open(self.pose_file, 'r', encoding='utf-8') as f:
            self.poses = json.load(f)
        
        print(f"✅ 已加载 {len(self.poses)} 个预设姿态")
    
    def get_pose(self, pose_name: str) -> list:
        """获取指定姿态的关节角度"""
        if pose_name not in self.poses:
            available = ", ".join(self.poses.keys())
            raise ValueError(f"姿态 '{pose_name}' 不存在!\n可用姿态: {available}")
        
        return self.poses[pose_name]['positions']
    
    def save_new_pose(self, pose_name: str, positions: list, description: str = "", arm: str = "left"):
        """保存新姿态到JSON文件"""
        self.poses[pose_name] = {
            "positions": positions,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "arm": arm,
            "description": description
        }
        
        with open(self.pose_file, 'w', encoding='utf-8') as f:
            json.dump(self.poses, f, indent=2, ensure_ascii=False)
        
        print(f"✅ 已保存新姿态: {pose_name}")


# ================= 3. Z轴移动计算器 =================
class ZAxisMoveCalculator:
    """基于预设姿态的Z轴移动计算"""
    
    # 关节限位 (从URDF提取)
    JOINT_LIMITS = {
        'left': [
            (-3.0892, 2.6704),   # shoulder_pitch
            (-1.5882, 2.2515),   # shoulder_roll
            (-2.618, 2.618),     # shoulder_yaw
            (-1.0472, 2.0944),   # elbow
            (-1.972222054, 1.972222054),  # wrist_roll
            (-1.614429558, 1.614429558),  # wrist_pitch
            (-1.614429558, 1.614429558)   # wrist_yaw
        ]
    }
    
    # 安全余量 (避免接近限位)
    JOINT_MARGIN = 0.1  # 弧度 (约5.7度)
    
    def __init__(self, urdf_file: str = "g1.urdf", arm: str = "left"):
        self.arm = arm
        tip_link = "left_hand_palm_link" if arm == "left" else "right_hand_palm_link"
        self.kinematic_chain = get_chain_from_urdf(urdf_file, "torso_link", tip_link)
        
        self.joint_names = [
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"
        ]
    
    def check_joint_limits(self, joints: np.ndarray) -> Tuple[bool, str]:
        """
        检查关节角度是否在限位内
        
        返回:
            (is_valid, violation_message)
        """
        limits = self.JOINT_LIMITS[self.arm]
        violations = []
        
        for i, (joint_val, (lower, upper)) in enumerate(zip(joints, limits)):
            # 应用安全余量
            safe_lower = lower + self.JOINT_MARGIN
            safe_upper = upper - self.JOINT_MARGIN
            
            if joint_val < safe_lower:
                violations.append(
                    f"{self.joint_names[i]}: {joint_val:.3f} < {safe_lower:.3f} (下限)"
                )
            elif joint_val > safe_upper:
                violations.append(
                    f"{self.joint_names[i]}: {joint_val:.3f} > {safe_upper:.3f} (上限)"
                )
        
        if violations:
            return False, "; ".join(violations)
        
        return True, "所有关节在安全范围内"
    
    def estimate_reachable_z_range(
        self, 
        current_joints: np.ndarray, 
        current_pos: np.ndarray,
        current_rot: np.ndarray,
        resolution: int = 20
    ) -> Tuple[float, float]:
        """
        🆕 动态估算当前姿态下的Z轴可达范围
        
        方法: 在当前位置基础上,尝试多个Z值,检查IK是否有解
        
        参数:
            current_joints: 当前关节角度
            current_pos: 当前末端位置
            current_rot: 当前末端姿态
            resolution: 采样分辨率
        
        返回:
            (z_min, z_max): 可达的Z轴范围
        """
        current_z = current_pos[2]
        
        # 初始搜索范围 (相对当前位置)
        search_range = (-0.5, 0.5)  # ±50cm
        
        # 向下搜索最小可达Z
        z_min = current_z
        for dz in np.linspace(0, search_range[0], resolution):
            test_pos = current_pos.copy()
            test_pos[2] += dz
            
            # 快速IK测试
            if self._is_position_reachable(test_pos, current_rot, current_joints):
                z_min = test_pos[2]
            else:
                break  # 遇到不可达点,停止搜索
        
        # 向上搜索最大可达Z
        z_max = current_z
        for dz in np.linspace(0, search_range[1], resolution):
            test_pos = current_pos.copy()
            test_pos[2] += dz
            
            if self._is_position_reachable(test_pos, current_rot, current_joints):
                z_max = test_pos[2]
            else:
                break
        
        return z_min, z_max
    
    def _is_position_reachable(
        self, 
        target_pos: np.ndarray, 
        target_rot: np.ndarray,
        seed_joints: np.ndarray
    ) -> bool:
        """
        快速检查目标位置是否可达
        
        策略: 执行IK求解,检查结果是否满足约束
        """
        try:
            # 构建种子状态
            seed_state = [0.0] + list(seed_joints) + [0.0]
            
            # 执行IK
            ik_solution = self.kinematic_chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_rot,
                orientation_mode="all",
                initial_position=seed_state,
                max_iter=50  # 降低迭代次数加快速度
            )
            
            # 提取关节角度
            ik_joints = np.array([ik_solution[i] for i in range(1, 8)])
            
            # 检查关节限位
            is_valid, _ = self.check_joint_limits(ik_joints)
            if not is_valid:
                return False
            
            # 验证位置误差
            verify_frame = self.kinematic_chain.forward_kinematics(ik_solution)
            verify_pos = verify_frame[:3, 3]
            pos_error = np.linalg.norm(verify_pos - target_pos)
            
            return pos_error < 0.01  # 10mm容差
            
        except Exception:
            return False
    
    def calculate_z_move(
        self, 
        current_joints: list, 
        delta_z: float, 
        verbose: bool = True,
        auto_adjust: bool = True
    ) -> dict:
        """
        计算Z轴移动后的新关节角度 (增强版)
        
        新增参数:
            auto_adjust: 如果目标超出范围,自动调整到边界
        """
        if len(current_joints) != 7:
            raise ValueError(f"关节数量错误! 期望7个,实际{len(current_joints)}个")
        
        current_joints = np.array(current_joints)
        current_state = [0.0] + list(current_joints) + [0.0]
        
        # ========== 步骤1: 计算当前位姿 ==========
        if verbose:
            print("\n" + "="*70)
            print("📌 步骤1: 正运动学计算当前末端位姿...")
        
        current_frame = self.kinematic_chain.forward_kinematics(current_state)
        current_pos = current_frame[:3, 3]
        current_rot = current_frame[:3, :3]
        
        if verbose:
            print(f"   当前位置: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}, Z={current_pos[2]:.4f}")
        
        # ========== 步骤2: 动态估算工作空间 ==========
        if verbose:
            print("\n📌 步骤2: 分析当前姿态的工作空间...")
        
        z_min, z_max = self.estimate_reachable_z_range(
            current_joints, current_pos, current_rot, resolution=15
        )
        
        if verbose:
            print(f"   当前Z轴可达范围: [{z_min*1000:.1f}mm, {z_max*1000:.1f}mm]")
            print(f"   可下移距离: {(current_pos[2]-z_min)*1000:.1f}mm")
            print(f"   可上移距离: {(z_max-current_pos[2])*1000:.1f}mm")
        
        # ========== 步骤3: 计算目标位置并验证 ==========
        target_z = current_pos[2] + delta_z
        
        if verbose:
            print(f"\n📌 步骤3: 目标位置验证...")
            print(f"   请求移动: {delta_z*1000:+.1f}mm")
            print(f"   目标Z轴: {target_z*1000:.1f}mm")
        
        # 检查是否超出范围
        if target_z < z_min or target_z > z_max:
            if auto_adjust:
                # 自动夹紧到边界
                adjusted_z = np.clip(target_z, z_min, z_max)
                actual_delta_z = adjusted_z - current_pos[2]
                
                if verbose:
                    print(f"   ⚠️ 目标超出范围,自动调整:")
                    print(f"      原始目标: {target_z*1000:.1f}mm")
                    print(f"      调整后: {adjusted_z*1000:.1f}mm")
                    print(f"      实际移动: {actual_delta_z*1000:+.1f}mm")
                
                target_z = adjusted_z
                delta_z = actual_delta_z
            else:
                error_msg = f"目标Z={target_z*1000:.1f}mm 超出可达范围 [{z_min*1000:.1f}, {z_max*1000:.1f}]mm"
                if verbose:
                    print(f"   ❌ {error_msg}")
                
                return {
                    'success': False,
                    'new_joints': None,
                    'current_pos': current_pos,
                    'target_pos': None,
                    'error_message': error_msg,
                    'workspace_limits': (z_min, z_max)
                }
        else:
            if verbose:
                print(f"   ✅ 目标位置在可达范围内")
        
        target_pos = current_pos.copy()
        target_pos[2] = target_z
        
        # ========== 步骤4: 逆运动学求解 ==========
        if verbose:
            print(f"\n📌 步骤4: 执行逆运动学求解...")
        
        ik_solution = self.kinematic_chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=current_rot,
            orientation_mode="all",
            initial_position=current_state
        )
        
        new_joints = np.array([ik_solution[i] for i in range(1, 8)])
        
        # ========== 步骤5: 验证结果 ==========
        if verbose:
            print(f"\n📌 步骤5: 验证IK解...")
        
        # 验证关节限位
        is_valid, limit_msg = self.check_joint_limits(new_joints)
        if not is_valid:
            if verbose:
                print(f"   ❌ 关节限位检查失败:")
                print(f"      {limit_msg}")
            
            return {
                'success': False,
                'new_joints': None,
                'error_message': f"关节限位违规: {limit_msg}",
                'workspace_limits': (z_min, z_max)
            }
        
        # 验证位置精度
        verify_frame = self.kinematic_chain.forward_kinematics(ik_solution)
        verify_pos = verify_frame[:3, 3]
        pos_error = np.linalg.norm(verify_pos - target_pos)
        
        if verbose:
            print(f"   验证位置: Z={verify_pos[2]*1000:.1f}mm")
            print(f"   位置误差: {pos_error*1000:.2f}mm")
            print(f"   关节限位: ✅ 通过")
            
            if pos_error < 0.001:
                print(f"   ✅ 位置精度优秀 (<1mm)")
            elif pos_error < 0.01:
                print(f"   ✅ 位置精度良好 (<10mm)")
            else:
                print(f"   ⚠️ 位置误差较大")
        
        return {
            'success': pos_error < 0.01,
            'new_joints': new_joints.tolist(),
            'current_pos': current_pos,
            'target_pos': target_pos,
            'verify_pos': verify_pos,
            'position_error': pos_error,
            'workspace_limits': (z_min, z_max),
            'actual_delta_z': delta_z  # 实际移动距离(可能被调整)
        }
    
    def print_joint_comparison(self, current_joints: list, new_joints: list):
        """打印关节角度对比"""
        print("\n" + "="*70)
        print("📊 关节角度对比")
        print("="*70)
        print(f"{'关节名称':<20} | {'当前角度':<12} | {'新角度':<12} | {'变化量':<12}")
        print("-" * 70)
        
        for i, name in enumerate(self.joint_names):
            current = current_joints[i]
            new = new_joints[i]
            diff = new - current
            
            diff_str = f"{diff:+.4f}"
            if abs(diff) > 0.5:  # 28度
                diff_str += " 🔴"
            elif abs(diff) > 0.2:  # 11度
                diff_str += " 🟡"
            
            print(f"{name:<20} | {current:8.4f}     | {new:8.4f}     | {diff_str:<12}")
        
        print("-" * 70)


# ================= 4. 主程序 =================
def main():
    """主函数"""
    
    # ========== 配置参数 ==========
    POSE_NAME = "test_phone_34"    # 📌 从此姿态开始
    DELTA_Z = 0.05                  # Z轴移动距离 (米)
    SAVE_RESULT = True               # 是否保存结果到JSON
    NEW_POSE_NAME = "test_phone_34_+_5cm"  # 新姿态名称
    ARM = "left"                     # 手臂
    
    print("="*70)
    print("🤖 G1机器人 - 基于预设姿态的Z轴移动计算")
    print("="*70)
    print(f"📋 初始姿态: {POSE_NAME}")
    print(f"📏 移动距离: {DELTA_Z*1000:+.1f} mm (Z轴)")
    print(f"💾 保存结果: {'是' if SAVE_RESULT else '否'}")
    if SAVE_RESULT:
        print(f"📝 新姿态名: {NEW_POSE_NAME}")
    print("="*70)
    
    try:
        # ========== 1. 加载姿态 ==========
        pose_loader = PoseLoader("../arm_control/saved_poses/left_arm_poses.json")
        current_joints = pose_loader.get_pose(POSE_NAME)
        
        print(f"\n✅ 已加载姿态 '{POSE_NAME}'")
        print(f"   关节角度: {[f'{x:.4f}' for x in current_joints]}")
        
        # ========== 2. 计算Z轴移动 ==========
        calculator = ZAxisMoveCalculator(urdf_file="g1.urdf", arm=ARM)
        result = calculator.calculate_z_move(current_joints, delta_z=DELTA_Z, verbose=True)
        
        if not result['success']:
            print("\n❌ IK求解精度不足,终止操作")
            sys.exit(1)
        
        # ========== 3. 显示对比 ==========
        calculator.print_joint_comparison(current_joints, result['new_joints'])
        
        # ========== 4. 输出可复制结果 ==========
        print("\n" + "="*70)
        print("📋 IK解算结果 (复制用)")
        print("="*70)
        
        new_joints = result['new_joints']
        
        # 紧凑格式
        print("\n# 紧凑格式(单行):")
        compact_str = "[" + ", ".join([f"{val:.6f}" for val in new_joints]) + "]"
        print(f"new_joints = {compact_str}")
        
        print("\n" + "="*70)
        
        # ========== 5. 保存到JSON (可选) ==========
        if SAVE_RESULT:
            description = f"从 {POSE_NAME} 沿Z轴移动 {DELTA_Z*1000:+.1f}mm 后的姿态"
            pose_loader.save_new_pose(
                pose_name=NEW_POSE_NAME,
                positions=new_joints,
                description=description,
                arm=ARM
            )
            print(f"\n✅ 已保存到 ../arm_control/saved_poses/left_arm_poses.json")
        
        print("\n🎉 计算完成!")
        sys.exit(0)
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()