import ikpy.chain
import ikpy.link
import numpy as np
import math
import xml.etree.ElementTree as ET

# ================= 1. URDF 解析工具 (保持不变) =================
def get_chain_from_urdf(urdf_file, base_link_name, tip_link_name):
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
    all_links = set([j['child_link'] for j in joints.values()] + [j['parent_link'] for j in joints.values()])
    if current_link not in all_links:
        raise ValueError(f"Link '{tip_link_name}' 未在 URDF 中找到")

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

    chain = ikpy.chain.Chain(ikpy_links, name="g1_left_arm", active_links_mask=active_mask)
    return chain

# ================= 2. 主程序逻辑 =================

def main():
    urdf_file = "g1.urdf"
    print("正在构建运动学链条...")
    left_arm_chain = get_chain_from_urdf(urdf_file, "torso_link", "left_hand_palm_link")
    print(f"链条构建成功,共 {len(left_arm_chain.links)} 个环节。")

    # ================= 3. 定义数据 =================
    
    # 【输入 A】初始状态 (当前机械臂姿态)
    # 从这里提取"姿态矩阵"作为约束
    prev_state_joints = [
      0.002999999999999989,
      0.168000000000001,
      -0.03099999999999975,
      -0.13399999999999967,
      1.41,
      0.027,
      -0.008
    ]
    seed_state = [0.0] + prev_state_joints + [0.0]

    # 【输入 B】Ground Truth 期望关节角度 (用于对比验证)
    target_ground_truth_joints = [
      0.002999999999999989,
      0.168000000000001,
      -0.03099999999999975,
      -0.13399999999999967,
      1.41,
      0.027,
      -0.008
    ]
    gt_state = [0.0] + target_ground_truth_joints + [0.0]

    # ================= 4. 提取 IK 所需参数 =================
    print("\n" + "="*40)
    print("📌 步骤1: 提取当前姿态作为约束...")
    
    # A. 从初始状态提取当前末端姿态 (3x3旋转矩阵)
    start_frame = left_arm_chain.forward_kinematics(seed_state)
    constraint_orientation = start_frame[:3, :3]  # ← 姿态锁定矩阵
    
    print(f"   ✅ 已锁定当前姿态:")
    print(f"      旋转矩阵形状: {constraint_orientation.shape}")
    print(f"      示例值 (第一行): {constraint_orientation[0]}")
    
    # # B. ⭐ 使用相机转换后的Torso坐标作为目标位置
    # target_pos_from_camera = np.array([(0.281,0.213,0.129)])  # ← 来自相机系统
    
    # print(f"\n📍 步骤2: 设定目标位置 (来自相机系统)...")
    # print(f"   目标坐标 (Torso系): {target_pos_from_camera}")
    
    # 原始相机坐标
    camera_pos_original = np.array([0.395,0.196,-0.144])
    # 测量误差 (单位: 米)
    # measurement_error = np.array([-0.02, -0.08, 0.25])  # 倒数第二行 (-5cm, -6cm, +25cm)
    measurement_error = np.array([-0.02, -0.08, 0.25])  # 倒数第一行(-5cm, -6cm, +25cm)
    # 应用误差后的目标坐标
    target_pos_from_camera = camera_pos_original + measurement_error  # ← 来自相机系统 + 测量误差

    print(f"\n   📷 原始相机坐标: {camera_pos_original}")
    print(f"   📏 测量误差: {measurement_error} (米)")
    print(f"   🎯 修正后坐标: {target_pos_from_camera}")

    # C. 验证fk求解Ground Truth结果与真实相机坐标的差异
    gt_frame = left_arm_chain.forward_kinematics(gt_state)
    gt_pos = gt_frame[:3, 3]
    pos_diff = np.linalg.norm(gt_pos - target_pos_from_camera)
    print(f"\n📐 坐标验证:")
    print(f"   fk求解Ground Truth位置: {gt_pos}")
    print(f"   相机采集位置:     {target_pos_from_camera}")
    print(f"   位置偏差:         {pos_diff*1000:.2f} mm")

    # ================= 5. 执行 IK (姿态保持 + 位置移动) =================
    print("\n" + "="*40)
    print("🔧 步骤3: 执行逆运动学求解...")
    print("   [约束条件]")
    print("   - 目标位置: 相机检测到的坐标 [0.260,0.247,-0.204] (Torso系)")
    print("   - 姿态限制: 保持当前手掌姿态不变")
    print("   - 求解模式: orientation_mode='all' (严格姿态约束)")

    ik_solution = left_arm_chain.inverse_kinematics(
        target_position=target_pos_from_camera,      # ← 直接使用相机坐标转换而来的torso坐标
        target_orientation=constraint_orientation,   # ← 锁定当前姿态
        orientation_mode="all",                      # ← 关键参数
        initial_position=seed_state
    )

    # ================= 6. 验证结果 =================
    print("\n" + "="*40)
    print("📊 步骤4: 验证求解结果...")
    
    # 验证1: 检查位置误差
    final_frame = left_arm_chain.forward_kinematics(ik_solution)
    final_pos = final_frame[:3, 3]
    final_rot = final_frame[:3, :3]
    
    pos_error = np.linalg.norm(final_pos - target_pos_from_camera)
    print(f"\n ik结果使用fk计算得到的坐标与目标坐标[位置验证]")
    print(f"   目标坐标: {target_pos_from_camera}")
    print(f"   实际到达: {final_pos}")
    print(f"   位置误差: {pos_error*1000:.2f} mm")
    print(f"   {'✅ 位置误差在可接受范围内' if pos_error < 0.02 else '⚠️ 位置误差较大，可能超出机械臂工作空间'}")
    # 验证2: 检查姿态保持情况
    orientation_error = np.linalg.norm(final_rot - constraint_orientation)
    print(f"\n[姿态验证]")
    print(f"   姿态偏差: {orientation_error:.6f}")
    if orientation_error < 0.01:
        print(f"   ✅ 姿态保持良好 (手掌方向未改变)")
    else:
        print(f"   ⚠️ 姿态有轻微变化 (可能超出机械臂工作空间)")
    
    # 验证3: 关节角度对比 (IK解 vs 初始状态 vs Ground Truth)
    print(f"\n[关节角度对比] (单位: 弧度)")
    joint_names = [
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw"
    ]
    
    print(f"{'关节名称':<25} | {'初始角度':<12} | {'IK解算':<12} | {'Ground Truth':<12} | {'IK-初始':<10} | {'IK-GT':<10}")
    print("-" * 110)
    
    total_error_vs_initial = 0
    total_error_vs_gt = 0
    
    for i, name in enumerate(joint_names):
        idx = i + 1
        initial_val = seed_state[idx]
        ik_val = ik_solution[idx]
        gt_val = gt_state[idx]
        
        diff_vs_initial = ik_val - initial_val
        diff_vs_gt = ik_val - gt_val
        
        total_error_vs_initial += abs(diff_vs_initial)
        total_error_vs_gt += abs(diff_vs_gt)
        
        # 标注显著差异
        diff_initial_str = f"{diff_vs_initial:+.4f}"
        diff_gt_str = f"{diff_vs_gt:+.4f}"
        
        if abs(diff_vs_initial) > 0.5:
            diff_initial_str += " 🔴"
        elif abs(diff_vs_initial) > 0.2:
            diff_initial_str += " 🟡"
            
        if abs(diff_vs_gt) > 0.1:
            diff_gt_str += " ⚠️"
        
        print(f"{name:<25} | {initial_val:8.4f}     | {ik_val:8.4f}     | {gt_val:8.4f}        | {diff_initial_str:<10} | {diff_gt_str:<10}")
    
    print("-" * 110)
    print(f"{'总误差':<25} | {'--':<12} | {'--':<12} | {'--':<12}        | {total_error_vs_initial:8.4f}   | {total_error_vs_gt:8.4f}")
    
    # ================= 7. 输出可复制的IK结果 =================
    print("\n" + "="*60)
    print("="*60)
    # 输出Python列表格式
    ik_joints = [ik_solution[i] for i in range(1, len(ik_solution)-1)]
    
    # 输出紧凑格式(单行)
    print("\n# 紧凑格式(单行):")
    compact_str = "[" + ", ".join([f"{val:.6f}" for val in ik_joints]) + "]"
    print(f"ik_result = {compact_str}")

    print("\n" + "="*60)

if __name__ == "__main__":
    main()
