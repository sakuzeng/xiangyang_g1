import numpy as np
from scipy.spatial.transform import Rotation as R

"""
宇树 G1-EDU 手臂正逆运动学计算
"""


class G1EDUArmKinematics:
    def __init__(self):
        """初始化左臂、右臂运动学链参数"""
        self.left_arm_joint_names = [
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint'
        ]

        self.right_arm_joint_names = [
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint'
        ]

        # 从URDF提取的关节参数，每个关节的origin变换: [x, y, z, rx, ry, rz] (平移和rpy旋转)
        self.left_arm_joint_origins = [
            [0.0039563, 0.10022, 0.23778, 0.27931, 5.4949e-05, -0.00019159],  # left_shoulder_pitch
            [0.0, 0.038, -0.013831, -0.27925, 0.0, 0.0],  # left_shoulder_roll
            [0.0, 0.00624, -0.1032, 0.0, 0.0, 0.0],  # left_shoulder_yaw
            [0.015783, 0.0, -0.080518, 0.0, 0.0, 0.0],  # left_elbow
            [0.1, 0.00188791, -0.01, 0.0, 0.0, 0.0],  # left_wrist_roll
            [0.038, 0.0, 0.0, 0.0, 0.0, 0.0],  # left_wrist_pitch
            [0.046, 0.0, 0.0, 0.0, 0.0, 0.0]  # left_wrist_yaw
        ]

        self.right_arm_joint_origins = [
            [0.0039563, -0.10021, 0.23778, -0.27931, 5.4949e-05, 0.00019159],  # right_shoulder_pitch
            [0, -0.038, -0.013831, 0.27925, 0, 0],  # right_shoulder_roll
            [0, -0.00624, -0.1032, 0.0, 0.0, 0.0],  # right_shoulder_yaw
            [0.015783, 0, -0.080518, 0.0, 0.0, 0.0],  # right_elbow
            [0.100, -0.00188791, -0.010, 0.0, 0.0, 0.0],  # right_wrist_roll
            [0.038, 0, 0, 0.0, 0.0, 0.0],  # right_wrist_pitch
            [0.046, 0, 0, 0.0, 0.0, 0.0]  # right_wrist_yaw
        ]

        # 关节旋转轴 (在子连杆坐标系中)
        self.left_arm_joint_axes = [
            [0, 1, 0],  # shoulder_pitch (绕Y轴)
            [1, 0, 0],  # shoulder_roll (绕X轴)
            [0, 0, 1],  # shoulder_yaw (绕Z轴)
            [0, 1, 0],  # elbow (绕Y轴)
            [1, 0, 0],  # wrist_roll (绕X轴)
            [0, 1, 0],  # wrist_pitch (绕Y轴)
            [0, 0, 1]  # wrist_yaw (绕Z轴)
        ]

        self.right_arm_joint_axes = [
            [0, 1, 0],  # shoulder_pitch (绕Y轴)
            [1, 0, 0],  # shoulder_roll (绕X轴)
            [0, 0, 1],  # shoulder_yaw (绕Z轴)
            [0, 1, 0],  # elbow (绕Y轴)
            [1, 0, 0],  # wrist_roll (绕X轴)
            [0, 1, 0],  # wrist_pitch (绕Y轴)
            [0, 0, 1]  # wrist_yaw (绕Z轴)
        ]

        # 关节限位 (弧度) - 从URDF提取
        self.left_arm_joint_limits = np.array([
            [-3.0892, 2.6704],  # left_shoulder_pitch
            [-1.5882, 2.2515],  # left_shoulder_roll
            [-2.618, 2.618],  # left_shoulder_yaw
            [-1.0472, 2.0944],  # left_elbow
            [-1.972222054, 1.972222054],  # left_wrist_roll
            [-1.614429558, 1.614429558],  # left_wrist_pitch
            [-1.614429558, 1.614429558]  # left_wrist_yaw
        ])

        self.right_arm_joint_limits = np.array([
            [-3.0892, 2.6704],  # right_shoulder_pitch
            [-2.2515, 1.5882],  # right_shoulder_roll
            [-2.618, 2.618],  # right_shoulder_yaw
            [-1.0472, 2.0944],  # right_elbow
            [-1.972222054, 1.972222054],  # right_wrist_roll
            [-1.614429558, 1.614429558],  # right_wrist_pitch
            [-1.614429558, 1.614429558]  # right_wrist_yaw
        ])

        # 关节零位
        self.joint_zero = np.zeros(7)
        # 关节中间位置
        self.left_arm_joint_midpoints = np.mean(self.left_arm_joint_limits, axis=1)
        self.right_arm_joint_midpoints = np.mean(self.right_arm_joint_limits, axis=1)

    def _get_transform_matrix(self, translation, rotation_rpy, axis=None, theta=0.0):
        """
        构建齐次变换矩阵

        Args:
            translation: [x, y, z] 平移
            rotation_rpy: [rx, ry, rz] 固定rpy旋转 (弧度)
            axis: 旋转轴 (用于关节旋转)
            theta: 关节角度 (弧度)

        Returns:
            4x4 齐次变换矩阵
        """
        # 固定旋转矩阵 (来自rpy)，xyz外旋
        rot_fixed = R.from_euler('xyz', rotation_rpy).as_matrix()

        # 关节旋转矩阵
        if axis is not None and abs(theta) > 1e-10:
            axis = np.array(axis) / np.linalg.norm(axis)  # 归一化
            rot_joint = R.from_rotvec(axis * theta).as_matrix()
            # 组合旋转: 固定旋转 * 关节旋转
            rot_total = rot_fixed @ rot_joint
        else:
            rot_total = rot_fixed

        # 构建齐次变换矩阵
        T = np.eye(4)
        T[:3, :3] = rot_total
        T[:3, 3] = translation

        return T

    def forward_kinematics_arm(self, joint_angles, arm):
        """
        计算手臂正运动学

        Args:
            joint_angles: 7个关节角度 (弧度)
            arm: 'left' 或 'right'，指定左臂或右臂

        Returns:
            T_total: 4x4 齐次变换矩阵 (torso_link -> wrist_yaw_link)
        """
        if arm == 'left':
            joint_origins = self.left_arm_joint_origins
            joint_axes = self.left_arm_joint_axes
        elif arm == 'right':
            joint_origins = self.right_arm_joint_origins
            joint_axes = self.right_arm_joint_axes
        else:
            raise ValueError(f"Invalid arm: {arm}. Must be 'left' or 'right'")

        T_total = np.eye(4)

        for i in range(7):
            origin = joint_origins[i]
            translation = origin[:3]
            rotation_rpy = origin[3:]
            axis = joint_axes[i]
            theta = joint_angles[i]

            T_i = self._get_transform_matrix(translation, rotation_rpy, axis, theta)
            T_total = T_total @ T_i

        return T_total


    def compute_jacobian(self, joint_angles, arm, delta=1e-6):
        """
        使用数值微分计算雅可比矩阵

        Args:
            joint_angles: 当前关节角度
            arm: 'left' 或 'right'，指定左臂或右臂
            delta: 微小扰动

        Returns:
            J: 6x7 雅可比矩阵
        """
        # 初始位姿
        T0 = self.forward_kinematics_arm(joint_angles, arm)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        J = np.zeros((6, 7))

        for i in range(7):
            # 扰动第i个关节
            angles_plus = joint_angles.copy()
            angles_plus[i] += delta

            # 计算扰动后的位姿
            T1 = self.forward_kinematics_arm(angles_plus, arm)
            p1 = T1[:3, 3]
            R1 = T1[:3, :3]

            # 位置雅可比 (平移变化)
            J[:3, i] = (p1 - p0) / delta

            # 姿态雅可比 (旋转变化)
            # 计算相对旋转矩阵
            R_rel = R1 @ R0.T
            # 将旋转矩阵转换为旋转向量 (轴角表示)
            rotvec = R.from_matrix(R_rel).as_rotvec()
            J[3:, i] = rotvec / delta

        return J

    def pose_error(self, T_desired_pose, T_current_pose):
        """
        计算位姿误差

        Args:
            T_desired_pose: 期望位姿 (4x4)
            T_current_pose: 当前位姿 (4x4)

        Returns:
            error: 6维误差向量 [位置误差, 姿态误差]
        """
        # 位置误差
        p_desired = T_desired_pose[:3, 3]
        p_current = T_current_pose[:3, 3]
        pos_error = p_desired - p_current

        # 姿态误差 (使用旋转向量表示)
        R_desired = T_desired_pose[:3, :3]
        R_current = T_current_pose[:3, :3]

        # 计算相对旋转矩阵
        R_rel = R_desired @ R_current.T
        # 转换为旋转向量
        rotvec_error = R.from_matrix(R_rel).as_rotvec()

        return np.concatenate([pos_error, rotvec_error])

    def check_joint_limits(self, joint_angles, arm):
        """检查关节角度是否在限位内"""
        if arm == 'left':
            arm_joint_limits = self.left_arm_joint_limits
        elif arm == 'right':
            arm_joint_limits = self.right_arm_joint_limits
        else:
            raise ValueError(f"Invalid arm: {arm}. Must be 'left' or 'right'")

        for i in range(7):
            if joint_angles[i] < arm_joint_limits[i, 0] or joint_angles[i] > arm_joint_limits[i, 1]:
                return False, i
        return True, -1

    def angle_distance(self, angles1, angles2):
        """计算两个关节角度向量的距离 (考虑周期性)"""
        diff = angles1 - angles2
        # 将角度差归一化到 [-π, π] 范围内
        diff = np.mod(diff + np.pi, 2 * np.pi) - np.pi
        return np.linalg.norm(diff)

    def inverse_kinematics(self, T_target_pose, arm,  current_joints=None,
                           max_iterations=100, tolerance=1e-4, lambda_damping=0.01,
                           num_random_seeds=20, use_current_as_seed=True):
        """
        逆运动学求解

        Args:
            T_target_pose: 期望位姿 (4x4 齐次变换矩阵，在torso_link坐标系下)
            arm: 'left' 或 'right'，指定左臂或右臂
            current_joints: 当前关节角度 (7维)，如果为None则使用中间位置
            max_iterations: 最大迭代次数
            tolerance: 收敛容差
            lambda_damping: 阻尼最小二乘法的阻尼系数
            num_random_seeds: 随机初始点数量
            use_current_as_seed: 是否使用当前位置作为初始点之一

        Returns:
            joint_angles: 7个关节角度 (弧度)，如果无解返回None
            success: 是否成功求解
            message: 状态信息
        """
        if current_joints is None:
            current_joints = self.joint_zero.copy()

        # 存储所有找到的解
        solutions = []

        # 生成初始点集合
        initial_guesses = []

        # 1. 使用当前位置作为初始点
        if use_current_as_seed:
            initial_guesses.append(current_joints.copy())

        if arm == 'left':
            joint_midpoints = self.left_arm_joint_midpoints
            joint_limits = self.left_arm_joint_limits
        elif arm == 'right':
            joint_midpoints = self.right_arm_joint_midpoints
            joint_limits = self.right_arm_joint_limits
        else:
            raise ValueError(f"Invalid arm: {arm}. Must be 'left' or 'right'")


        # 2. 使用关节中间位置
        initial_guesses.append(joint_midpoints.copy())

        # 3. 生成随机初始点
        for _ in range(num_random_seeds):
            random_guess = np.zeros(7)
            for i in range(7):
                lower, upper = joint_limits[i]
                random_guess[i] = np.random.uniform(lower, upper)
            initial_guesses.append(random_guess)

        # 对每个初始点进行迭代求解
        for init_guess in initial_guesses:
            theta = init_guess.copy()
            prev_error_norm = float('inf')
            stuck_count = 0

            for iteration in range(max_iterations):
                # 计算当前位姿
                T_current_pose = self.forward_kinematics_arm(theta, arm)

                # 计算误差
                error = self.pose_error(T_target_pose, T_current_pose)
                error_norm = np.linalg.norm(error)

                # 检查是否收敛
                if error_norm < tolerance:
                    # 检查关节限位
                    within_limits, violating_joint = self.check_joint_limits(theta, arm)
                    if within_limits:
                        solutions.append(theta.copy())
                    break

                # 检查是否陷入局部极小值
                if abs(prev_error_norm - error_norm) < 1e-6:
                    stuck_count += 1
                else:
                    stuck_count = 0

                if stuck_count > 10:
                    break  # 陷入局部极小值，尝试下一个初始点

                prev_error_norm = error_norm

                # 计算雅可比矩阵
                J = self.compute_jacobian(theta, arm)

                # 使用阻尼最小二乘法求解关节角度增量
                # Δθ = J^T (J J^T + λ^2 I)^(-1) e
                I = np.eye(6)
                JT = J.T
                JJT = J @ JT
                damping_matrix = lambda_damping ** 2 * I

                # 避免奇异，使用伪逆
                try:
                    # 直接求解线性系统
                    delta_theta = JT @ np.linalg.solve(JJT + damping_matrix, error)
                except np.linalg.LinAlgError:
                    # 如果求解失败，使用SVD伪逆
                    U, S, Vh = np.linalg.svd(J, full_matrices=False)
                    S_inv = S / (S ** 2 + lambda_damping ** 2)
                    delta_theta = Vh.T @ (S_inv * (U.T @ error))

                # 更新关节角度
                theta = theta + delta_theta

                # 限制关节角度在限位内
                for i in range(7):
                    lower, upper = joint_limits[i]
                    if theta[i] < lower:
                        theta[i] = lower
                    elif theta[i] > upper:
                        theta[i] = upper

            # 如果迭代到最后一次，检查是否满足精度
            T_final_pose = self.forward_kinematics_arm(theta, arm)
            error_final = self.pose_error(T_target_pose, T_final_pose)
            if np.linalg.norm(error_final) < tolerance:
                within_limits, violating_joint = self.check_joint_limits(theta, arm)
                if within_limits:
                    solutions.append(theta.copy())

        # 如果没有找到解
        if not solutions:
            return None, False, "No solution found within joint limits and tolerance"

        # 选择距离当前位置最近的解 (最短路径)
        best_solution = None
        min_distance = float('inf')

        for sol in solutions:
            distance = self.angle_distance(sol, current_joints)
            if distance < min_distance:
                min_distance = distance
                best_solution = sol

        return best_solution, True, f"Found solution with distance {min_distance:.6f} from current position"

    def create_target_pose(self, position, orientation):
        """
        从位置和姿态创建齐次变换矩阵

        Args:
            position: [x, y, z] 位置 (在torso_link坐标系下)
            orientation: 姿态，可以是以下格式之一:
                - [rx, ry, rz] 欧拉角 (弧度，XYZ顺序，外旋)
                - [qw, qx, qy, qz] 四元数
                - 3x3 旋转矩阵

        Returns:
            4x4 齐次变换矩阵
        """
        T = np.eye(4)
        T[:3, 3] = position

        if isinstance(orientation, np.ndarray):
            if orientation.shape == (3,):  # 欧拉角
                rot = R.from_euler('xyz', orientation).as_matrix()
            elif orientation.shape == (4,):  # 四元数
                rot = R.from_quat(orientation).as_matrix()
            elif orientation.shape == (3, 3):  # 旋转矩阵
                rot = orientation
            else:
                raise ValueError("Invalid orientation format")
        elif isinstance(orientation, R):  # Rotation对象
            rot = orientation.as_matrix()
        else:
            raise ValueError("Invalid orientation type")

        T[:3, :3] = rot
        return T


if __name__ == '__main__':
    # 创建运动学求解器
    solver = G1EDUArmKinematics()

    # 定义一个目标位姿 (在torso_link坐标系下)，是 left_wrist_yaw_link 坐标系原点在 torso_link 坐标系中的位姿
    target_position = np.array([0.3, 0.2, 0.1])  # 单位：米
    target_orientation = np.array([0.1, 0.2, 0.3])  # 欧拉角 (弧度)

    T_target_pose = solver.create_target_pose(target_position, target_orientation)

    # 设置当前7个关节角，弧度制
    current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 明确对哪个手臂进行运动学计算
    arm = 'left'

    # 求解逆运动学
    # 修改参数能够加快计算速率
    joint_angles, success, message = solver.inverse_kinematics(
        T_target_pose=T_target_pose,
        arm=arm,
        current_joints=current_joints,
        max_iterations=50,
        tolerance=1e-4,
        lambda_damping=0.1,
        num_random_seeds=30
    )


    if success:
        print("逆运动学求解成功!")
        print(f"状态: {message}")
        print("关节角度 (弧度):")
        for i, (name, angle) in enumerate(zip(solver.left_arm_joint_names, joint_angles)):
            print(f"  {name}: {angle:.6f} rad ({np.degrees(angle):.2f} deg)")

        # 验证解
        # 正向运动学求解
        T_reached = solver.forward_kinematics_arm(joint_angles, arm)
        print("\n验证:")
        print(f"目标位置: {target_position}")
        print(f"达到位置: {T_reached[:3, 3]}")
        print(f"位置误差: {np.linalg.norm(target_position - T_reached[:3, 3]):.6f}")

        # 检查关节限位
        within_limits, violating_joint = solver.check_joint_limits(joint_angles, arm)
        if within_limits:
            print("所有关节都在限位内")
        else:
            print(f"警告: 关节超出限位")

        # 假设点 P 在 left/right_wrist_yaw_link 坐标系下的坐标
        P_wrist = np.array([0.05, 0.02, 0.03, 1])  # 齐次坐标
        # 转换到 torso_link 坐标系
        P_torso = T_reached @ P_wrist
        print(f"点 P 在 torso_link 坐标系下的坐标: {P_torso[:3]}")

    else:
        print("逆运动学求解失败!")
        print(f"原因: {message}")




