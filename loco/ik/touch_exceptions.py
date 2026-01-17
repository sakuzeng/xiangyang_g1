class TouchSystemError(Exception):
    """Base exception for touch system errors."""
    pass

class CameraError(TouchSystemError):
    """Raised when camera initialization or frame acquisition fails.
    • 摄像头USB连接断开 • 摄像头被其他进程占用 • get_frames() 返回空数据"""
    pass

class YoloServiceError(TouchSystemError):
    """Raised when YOLO service call fails or returns invalid response.
    推理服务器挂了 • 网络不通 (IP错误/断网) • HTTP超时或返回 500 错误"""
    pass

class TargetNotFoundError(TouchSystemError):
    """Raised when the target screen or region is not found.
    屏幕被遮挡 • 目标编号不存在 • 环境光太暗导致识别失败"""
    pass

class DepthAcquisitionError(TouchSystemError):
    """Raised when valid depth information cannot be acquired.
    目标区域反光（深度为0） • 目标超出深度相机量程 • Torso Z 验证失败 （计算出的点在空间中不合理，比如飘在天上）"""
    pass

class IKSolutionError(TouchSystemError):
    """Raised when Inverse Kinematics solver fails to find a solution.
    目标点超出了机械臂的长度（够不着） • 目标点虽然够得着，但需要的手腕角度奇异 • 计算出的解位置误差 > 5cm"""
    pass

class SafetyLimitError(TouchSystemError):
    """Raised when target coordinates are out of safe range.
    计算出的 Torso X/Y 坐标超出了你设定的安全矩形范围（例如 X > 0.38）"""
    pass

class RobotControlError(TouchSystemError):
    """Raised when robot arm or hand control fails.
    • 机器人底层通信断开 • 关节过热保护 • 运动指令发送失败 • 初始姿态校验失败"""
    pass