"""
机器人状态管理模块 - 完整版 v3
特性:
- 单例客户端管理（手臂和灵巧手都分离左右）
- 线程安全控制
- 自动冲突检测
- 上下文管理器
"""
import threading
import time
from typing import Optional, Dict
from contextlib import contextmanager


class RobotStateManager:
    """单例模式的机器人状态管理器"""
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        # 🆕 唯一的客户端实例（手臂和灵巧手都分左右）
        self._arm_client = None  # 手臂客户端只有一个，但控制左右手臂分离
        self._hand_clients: Dict[str, any] = {}  # {'left': Dex3Client, 'right': Dex3Client}
        self._loco_client = None
        
        # 🆕 控制锁（手臂和灵巧手都分左右）
        self._arm_control_locks: Dict[str, threading.Lock] = {
            'left': threading.Lock(),
            'right': threading.Lock()
        }
        self._hand_control_locks: Dict[str, threading.Lock] = {
            'left': threading.Lock(),
            'right': threading.Lock()
        }
        self._movement_lock = threading.Lock()
        
        # 🆕 控制状态（手臂和灵巧手都分左右）
        self.is_arm_controlling: Dict[str, bool] = {
            'left': False,
            'right': False
        }
        self.is_hand_controlling: Dict[str, bool] = {
            'left': False,
            'right': False
        }
        self.is_moving = False
        
        # 🆕 当前控制者信息（手臂和灵巧手都分左右）
        self._arm_controller_names: Dict[str, Optional[str]] = {
            'left': None,
            'right': None
        }
        self._hand_controller_names: Dict[str, Optional[str]] = {
            'left': None,
            'right': None
        }
        self._movement_controller_name = None
        
        self.debug_mode = True
        self._initialized = True
    
    def _log(self, message: str):
        if self.debug_mode:
            timestamp = time.strftime('%H:%M:%S')
            print(f"[StateManager {timestamp}] {message}")
    
    # ========== 单例客户端管理 ==========
    
    def get_or_create_arm_client(self, interface: str = "eth0"):
        """
        获取或创建唯一的手臂客户端（线程安全）
        
        注意: 手臂客户端只有一个实例，控制双臂14DOF
              但左右手臂的控制权是分离的
        """
        with self._lock:
            if self._arm_client is None:
                from unitree_sdk2py.arm.arm_client import G1ArmClient, G1ArmConfig
                self._log("🆕 创建手臂客户端（双臂14DOF）")
                config = G1ArmConfig(enable_waist_control=False)
                self._arm_client = G1ArmClient(interface=interface, config=config)
            else:
                self._log("♻️  复用现有手臂客户端")
            return self._arm_client
    
    def get_or_create_hand_client(self, hand: str = "left", interface: str = "eth0"):
        """
        获取或创建灵巧手客户端（线程安全，左右手分离）
        
        参数:
            hand: 'left' 或 'right'
            interface: 网络接口名称
        """
        if hand not in ['left', 'right']:
            raise ValueError(f"❌ 无效的手参数: {hand}，必须是 'left' 或 'right'")
        
        with self._lock:
            if hand not in self._hand_clients or self._hand_clients[hand] is None:
                from unitree_sdk2py.dex3.dex3_client import Dex3Client
                self._log(f"🆕 创建 {hand.upper()} 手灵巧手客户端")
                self._hand_clients[hand] = Dex3Client(hand=hand, interface=interface)
            else:
                self._log(f"♻️  复用现有 {hand.upper()} 手灵巧手客户端")
            return self._hand_clients[hand]
    
    # ========== 安全控制上下文 ==========
    
    @contextmanager
    def safe_arm_control(self, arm: str = "left", source: str = "unknown", timeout: float = 5.0):
        """
        安全的手臂控制上下文（左右手臂分离）
        
        参数:
            arm: 'left' 或 'right'
            source: 控制来源标识
            timeout: 超时时间（秒）
        
        特性:
        - 自动加锁/解锁指定手臂
        - 超时保护
        - 冲突检测
        """
        if arm not in ['left', 'right']:
            raise ValueError(f"❌ 无效的手臂参数: {arm}")
        
        lock = self._arm_control_locks[arm]
        acquired = lock.acquire(timeout=timeout)
        
        if not acquired:
            raise RuntimeError(
                f"❌ 无法获取 {arm.upper()} 手臂控制权（超时{timeout}秒）\n"
                f"   当前控制者: {self._arm_controller_names[arm]}"
            )
        
        try:
            self._arm_controller_names[arm] = source
            self.is_arm_controlling[arm] = True
            self._log(f"🔒 {source} 获得 {arm.upper()} 手臂控制权")
            yield self._arm_client
        finally:
            self.is_arm_controlling[arm] = False
            self._arm_controller_names[arm] = None
            lock.release()
            self._log(f"🔓 {source} 释放 {arm.upper()} 手臂控制权")
    
    @contextmanager
    def safe_hand_control(self, hand: str = "left", source: str = "unknown", timeout: float = 5.0):
        """
        安全的灵巧手控制上下文（左右手分离）
        
        参数:
            hand: 'left' 或 'right'
            source: 控制来源标识
            timeout: 超时时间（秒）
        """
        if hand not in ['left', 'right']:
            raise ValueError(f"❌ 无效的手参数: {hand}")
        
        lock = self._hand_control_locks[hand]
        acquired = lock.acquire(timeout=timeout)
        
        if not acquired:
            raise RuntimeError(
                f"❌ 无法获取 {hand.upper()} 手控制权（超时{timeout}秒）\n"
                f"   当前控制者: {self._hand_controller_names[hand]}"
            )
        
        try:
            self._hand_controller_names[hand] = source
            self.is_hand_controlling[hand] = True
            self._log(f"🔒 {source} 获得 {hand.upper()} 手控制权")
            yield self._hand_clients.get(hand)
        finally:
            self.is_hand_controlling[hand] = False
            self._hand_controller_names[hand] = None
            lock.release()
            self._log(f"🔓 {source} 释放 {hand.upper()} 手控制权")
    
    @contextmanager
    def safe_dual_arm_control(self, source: str = "unknown", timeout: float = 5.0):
        """
        安全的双手臂控制上下文（同时控制左右手臂）
        
        参数:
            source: 控制来源标识
            timeout: 超时时间（秒）
        
        使用场景: 需要协调控制双臂的动作（如拥抱、举手等）
        """
        # 按固定顺序获取锁（避免死锁）
        left_lock = self._arm_control_locks['left']
        right_lock = self._arm_control_locks['right']
        
        left_acquired = left_lock.acquire(timeout=timeout)
        if not left_acquired:
            raise RuntimeError(f"❌ 无法获取左臂控制权（超时{timeout}秒）")
        
        try:
            right_acquired = right_lock.acquire(timeout=timeout)
            if not right_acquired:
                raise RuntimeError(f"❌ 无法获取右臂控制权（超时{timeout}秒）")
            
            try:
                self._arm_controller_names['left'] = source
                self._arm_controller_names['right'] = source
                self.is_arm_controlling['left'] = True
                self.is_arm_controlling['right'] = True
                self._log(f"🔒 {source} 获得双臂控制权")
                yield self._arm_client
            finally:
                self.is_arm_controlling['left'] = False
                self.is_arm_controlling['right'] = False
                self._arm_controller_names['left'] = None
                self._arm_controller_names['right'] = None
                right_lock.release()
                self._log(f"🔓 {source} 释放双臂控制权")
        finally:
            left_lock.release()
    
    # ========== 状态查询 ==========
    
    def get_status_string(self) -> str:
        """获取详细状态字符串"""
        left_arm_status = (f"🔴{self._arm_controller_names['left']}" 
                          if self.is_arm_controlling['left'] else "⚪空闲")
        right_arm_status = (f"🔴{self._arm_controller_names['right']}" 
                           if self.is_arm_controlling['right'] else "⚪空闲")
        
        left_hand_status = (f"🔴{self._hand_controller_names['left']}" 
                           if self.is_hand_controlling['left'] else "⚪空闲")
        right_hand_status = (f"🔴{self._hand_controller_names['right']}" 
                            if self.is_hand_controlling['right'] else "⚪空闲")
        
        move_status = f"🔴{self._movement_controller_name}" if self.is_moving else "⚪静止"
        
        return (f"左臂:{left_arm_status} | 右臂:{right_arm_status} | "
                f"左手:{left_hand_status} | 右手:{right_hand_status} | "
                f"运动:{move_status}")
    
    def is_any_limb_controlling(self) -> bool:
        """检查是否有任何肢体正在控制"""
        return (self.is_arm_controlling['left'] or 
                self.is_arm_controlling['right'] or
                self.is_hand_controlling['left'] or 
                self.is_hand_controlling['right'])
    
    def is_arm_side_controlling(self, side: str) -> bool:
        """检查指定侧（左或右）的手臂和手是否正在控制"""
        if side not in ['left', 'right']:
            return False
        return self.is_arm_controlling[side] or self.is_hand_controlling[side]
    
    def get_arm_status(self, arm: str) -> str:
        """获取指定手臂的状态"""
        if arm not in ['left', 'right']:
            return "❌ 无效"
        
        if self.is_arm_controlling[arm]:
            return f"🔴 控制中 ({self._arm_controller_names[arm]})"
        else:
            return "⚪ 空闲"
    
    def get_hand_status(self, hand: str) -> str:
        """获取指定手的状态"""
        if hand not in ['left', 'right']:
            return "❌ 无效"
        
        if self.is_hand_controlling[hand]:
            return f"🔴 控制中 ({self._hand_controller_names[hand]})"
        else:
            return "⚪ 空闲"
    
    # ========== 紧急停止 ==========
    
    def emergency_stop_all(self) -> bool:
        """紧急停止所有控制"""
        self._log("🚨 执行紧急停止...")
        success = True
        
        # 停止双臂（只需调用一次，因为只有一个客户端）
        if self._arm_client:
            # 需要同时持有两个锁
            with self._arm_control_locks['left']:
                with self._arm_control_locks['right']:
                    try:
                        self._arm_client.stop_control()
                        self._log("✅ 双臂已停止")
                    except Exception as e:
                        self._log(f"❌ 双臂停止失败: {e}")
                        success = False
        
        # 停止左右手
        for hand in ['left', 'right']:
            if hand in self._hand_clients and self._hand_clients[hand]:
                lock = self._hand_control_locks[hand]
                with lock:
                    try:
                        self._hand_clients[hand].stop_control()
                        self._log(f"✅ {hand.upper()} 手已停止")
                    except Exception as e:
                        self._log(f"❌ {hand.upper()} 手停止失败: {e}")
                        success = False
        
        self.reset_all_states()
        return success
    
    def emergency_stop_arm(self, arm: str) -> bool:
        """
        紧急停止指定手臂
        
        注意: 由于手臂客户端控制双臂，停止一侧会影响整体
              建议只在单臂控制场景使用
        """
        if arm not in ['left', 'right']:
            self._log(f"❌ 无效的手臂参数: {arm}")
            return False
        
        self._log(f"🚨 紧急停止 {arm.upper()} 手臂...")
        
        if self._arm_client:
            lock = self._arm_control_locks[arm]
            with lock:
                try:
                    # 注意：这会停止整个手臂客户端
                    self._arm_client.stop_control()
                    self.is_arm_controlling[arm] = False
                    self._arm_controller_names[arm] = None
                    self._log(f"✅ {arm.upper()} 手臂已停止")
                    return True
                except Exception as e:
                    self._log(f"❌ {arm.upper()} 手臂停止失败: {e}")
                    return False
        else:
            self._log(f"⚠️  手臂客户端未创建")
            return False
    
    def emergency_stop_hand(self, hand: str) -> bool:
        """紧急停止指定手"""
        if hand not in ['left', 'right']:
            self._log(f"❌ 无效的手参数: {hand}")
            return False
        
        self._log(f"🚨 紧急停止 {hand.upper()} 手...")
        
        if hand in self._hand_clients and self._hand_clients[hand]:
            lock = self._hand_control_locks[hand]
            with lock:
                try:
                    self._hand_clients[hand].stop_control()
                    self.is_hand_controlling[hand] = False
                    self._hand_controller_names[hand] = None
                    self._log(f"✅ {hand.upper()} 手已停止")
                    return True
                except Exception as e:
                    self._log(f"❌ {hand.upper()} 手停止失败: {e}")
                    return False
        else:
            self._log(f"⚠️  {hand.upper()} 手客户端未创建")
            return False
    
    def reset_all_states(self):
        """重置所有状态"""
        self.is_arm_controlling['left'] = False
        self.is_arm_controlling['right'] = False
        self.is_hand_controlling['left'] = False
        self.is_hand_controlling['right'] = False
        self.is_moving = False
        
        self._arm_controller_names['left'] = None
        self._arm_controller_names['right'] = None
        self._hand_controller_names['left'] = None
        self._hand_controller_names['right'] = None
        self._movement_controller_name = None
        
        self._log("🔄 所有控制状态已重置")
    
    def reset_arm_state(self, arm: str):
        """重置指定手臂的状态"""
        if arm in ['left', 'right']:
            self.is_arm_controlling[arm] = False
            self._arm_controller_names[arm] = None
            self._log(f"🔄 {arm.upper()} 手臂状态已重置")
    
    def reset_hand_state(self, hand: str):
        """重置指定手的状态"""
        if hand in ['left', 'right']:
            self.is_hand_controlling[hand] = False
            self._hand_controller_names[hand] = None
            self._log(f"🔄 {hand.upper()} 手状态已重置")


# 全局单例
robot_state = RobotStateManager()