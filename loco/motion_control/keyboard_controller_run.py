"""
Unitree G-1 机器人键盘遥控程序 - 优化版

优化点:
1. 渐进加减速 - 平滑的速度过渡
2. 速度变化检测 - 仅在速度改变时发送指令
3. 改进的急停逻辑 - 提前检测冲突
4. 更好的状态显示 - 实时显示加速度
"""
from __future__ import annotations

import argparse
import time
import curses
import os
import sys

try:
    from pynput.keyboard import Listener, Key, KeyCode
except ModuleNotFoundError as exc:
    raise SystemExit(
        "需要 'pynput' 依赖包。\n"
        "请使用以下命令安装: pip install pynput"
    ) from exc

from hanger_boot_sequence_run import hanger_boot_sequence

# 导入状态管理器
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from common.robot_state_manager import robot_state

# --- 🆕 优化的参数 ---
MAX_LINEAR_VEL = 0.3      # m/s: 最大线速度
MAX_ANGULAR_VEL = 0.5     # rad/s: 最大角速度

ACCEL_RATE = 0.4          # m/s²: 加速度（每秒增加的速度）
DECEL_RATE = 0.4          # m/s²: 减速度（更快的减速）

CONTROL_FREQ = 50         # Hz: 控制频率（提高到50Hz，更平滑）
CONTROL_DT = 1.0 / CONTROL_FREQ  # 控制周期 0.02秒

SEND_PERIOD = 0.05        # 🆕 降低发送周期到20Hz（仅在速度变化时发送）


def clamp(value: float, limit: float) -> float:
    """将速度值限制在 [-limit, +limit] 范围内"""
    return max(-limit, min(limit, value))


def smooth_approach(current: float, target: float, rate: float, dt: float) -> float:
    """
    🆕 平滑接近目标值（渐进加减速）
    
    Args:
        current: 当前值
        target: 目标值
        rate: 变化率（加速度）
        dt: 时间步长
    
    Returns:
        新的当前值
    """
    delta = target - current
    max_change = rate * dt
    
    if abs(delta) <= max_change:
        return target
    else:
        return current + max_change if delta > 0 else current - max_change


def drive_loop(stdscr: "curses._CursesWindow", bot) -> None:
    """
    🆕 优化的键盘遥控主循环
    
    优化点:
    - 渐进加减速，平滑控制
    - 仅在速度变化时发送指令
    - 提前检测手臂冲突
    """
    # 初始化 Curses HUD
    curses.cbreak()
    stdscr.nodelay(True)

    # 🆕 当前速度和目标速度分离
    current_vx = current_vy = current_omega = 0.0
    target_vx = target_vy = target_omega = 0.0
    
    # 🆕 上次发送的速度（用于检测变化）
    last_sent_vx = last_sent_vy = last_sent_omega = 0.0
    
    last_control = time.time()
    last_send = time.time()
    
    # 🆕 手臂冲突检测标志
    limb_conflict_detected = False

    # --- pynput 键盘监听器设置 ---
    pressed_keys: set[object] = set()

    def _on_press(key):
        if isinstance(key, KeyCode) and key.char is not None:
            pressed_keys.add(key.char.lower())
        else:
            pressed_keys.add(key)

    def _on_release(key):
        if isinstance(key, KeyCode) and key.char is not None:
            pressed_keys.discard(key.char.lower())
        else:
            pressed_keys.discard(key)

    listener = Listener(on_press=_on_press, on_release=_on_release)
    listener.start()

    def key(name: str) -> bool:
        return name in pressed_keys

    try:
        while True:
            now = time.time()
            dt = now - last_control
            
            # 🆕 1. 根据按键更新**目标速度**（而非直接速度）
            if dt >= CONTROL_DT:
                last_control = now
                
                # 前后移动
                if key("w") and not key("s"):
                    target_vx = MAX_LINEAR_VEL
                elif key("s") and not key("w"):
                    target_vx = -MAX_LINEAR_VEL
                else:
                    target_vx = 0.0

                # 左右平移
                if key("q") and not key("e"):
                    target_vy = MAX_LINEAR_VEL
                elif key("e") and not key("q"):
                    target_vy = -MAX_LINEAR_VEL
                else:
                    target_vy = 0.0

                # 旋转
                if key("a") and not key("d"):
                    target_omega = MAX_ANGULAR_VEL
                elif key("d") and not key("a"):
                    target_omega = -MAX_ANGULAR_VEL
                else:
                    target_omega = 0.0

                # 🆕 2. 平滑接近目标速度（渐进加减速）
                # 判断是加速还是减速
                rate_vx = DECEL_RATE if abs(target_vx) < abs(current_vx) else ACCEL_RATE
                rate_vy = DECEL_RATE if abs(target_vy) < abs(current_vy) else ACCEL_RATE
                rate_omega = DECEL_RATE if abs(target_omega) < abs(current_omega) else ACCEL_RATE
                
                current_vx = smooth_approach(current_vx, target_vx, rate_vx, dt)
                current_vy = smooth_approach(current_vy, target_vy, rate_vy, dt)
                current_omega = smooth_approach(current_omega, target_omega, rate_omega, dt)

            # 🆕 3. 仅在速度变化时发送指令
            if now - last_send >= SEND_PERIOD:
                last_send = now
                
                # 🆕 检测速度是否真的变化（避免频繁发送相同指令）
                vel_changed = (
                    abs(current_vx - last_sent_vx) > 0.01 or
                    abs(current_vy - last_sent_vy) > 0.01 or
                    abs(current_omega - last_sent_omega) > 0.01
                )
                
                # 🆕 提前检测手臂冲突（在开始移动时）
                moving = (abs(current_vx) > 0.01 or abs(current_vy) > 0.01 or abs(current_omega) > 0.01)
                
                if moving and robot_state.is_any_limb_controlling() and not limb_conflict_detected:
                    stdscr.addstr(2, 0, "⚠️ 检测到手臂/灵巧手控制中，正在停止...   ")
                    stdscr.refresh()
                    
                    if robot_state.emergency_stop_all():
                        time.sleep(0.3)
                        stdscr.addstr(2, 0, "✅ 手臂/灵巧手已停止，可以安全移动      ")
                        limb_conflict_detected = True
                    else:
                        stdscr.addstr(2, 0, "❌ 无法停止手臂/灵巧手，移动受阻      ")
                        current_vx = current_vy = current_omega = 0.0
                        target_vx = target_vy = target_omega = 0.0
                    
                    stdscr.refresh()
                    time.sleep(0.3)
                
                # 🆕 重置冲突检测标志（当停止移动时）
                if not moving:
                    limb_conflict_detected = False
                
                # 🆕 仅在速度变化时发送指令
                if vel_changed or moving:
                    bot.Move(current_vx, current_vy, current_omega, continous_move=True)
                    last_sent_vx = current_vx
                    last_sent_vy = current_vy
                    last_sent_omega = current_omega

                # 🆕 4. 更新 HUD 显示
                stdscr.erase()
                stdscr.addstr(0, 0, "🎮 G1 键盘遥控 - WASD控制 | Ctrl+C退出")
                stdscr.addstr(1, 0, f"📊 状态: {robot_state.get_status_string()}")
                
                # 🆕 显示当前速度和目标速度
                stdscr.addstr(3, 0, f"📍 当前速度: vx={current_vx:+.2f} m/s  vy={current_vy:+.2f} m/s  omega={current_omega:+.2f} rad/s")
                stdscr.addstr(4, 0, f"🎯 目标速度: vx={target_vx:+.2f} m/s  vy={target_vy:+.2f} m/s  omega={target_omega:+.2f} rad/s")
                
                # 🆕 显示加速状态
                if abs(current_vx - target_vx) > 0.01 or abs(current_vy - target_vy) > 0.01 or abs(current_omega - target_omega) > 0.01:
                    if abs(target_vx) > abs(current_vx) or abs(target_vy) > abs(current_vy) or abs(target_omega) > abs(current_omega):
                        stdscr.addstr(5, 0, "🚀 加速中...")
                    else:
                        stdscr.addstr(5, 0, "🛑 减速中...")
                else:
                    stdscr.addstr(5, 0, "✅ 速度稳定")
                
                # 警告信息
                if robot_state.is_any_limb_controlling():
                    try:
                        stdscr.addstr(6, 0, "⚠️  警告: 手臂/灵巧手激活中", curses.A_BOLD)
                    except:
                        pass
                
                stdscr.refresh()

            time.sleep(0.001)  # 🆕 降低循环延迟（1ms），提高响应速度

    finally:
        listener.stop()
        # 🆕 优雅停止：渐进减速到0
        print("\n正在优雅停止...")
        for _ in range(10):
            current_vx = smooth_approach(current_vx, 0.0, DECEL_RATE, 0.1)
            current_vy = smooth_approach(current_vy, 0.0, DECEL_RATE, 0.1)
            current_omega = smooth_approach(current_omega, 0.0, DECEL_RATE, 0.1)
            bot.Move(current_vx, current_vy, current_omega, continous_move=True)
            time.sleep(0.1)
        
        bot.StopMove()
        robot_state.reset_all_states()


def main() -> None:
    """程序主入口"""
    parser = argparse.ArgumentParser(description="Unitree G-1 键盘遥控程序（优化版）")
    parser.add_argument("--iface", default="eth0", help="连接到机器人的网络接口")
    args = parser.parse_args()

    bot = hanger_boot_sequence(iface=args.iface)
    curses.wrapper(drive_loop, bot)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被中断 – 正在停止...")
        try:
            print("请确认机器人已正确悬挂")
            robot_state.emergency_stop_all()
        except Exception:
            pass
