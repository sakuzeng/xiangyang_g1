#!/usr/bin/env python3
"""
phone_touch_interface.py
========================

手机触摸功能的高层接口封装。
提供简单的 touch_target 函数，自动处理初始化、模式识别和错误传播。
"""

import sys
import os
from typing import Optional, Tuple

# 添加路径以确保能导入依赖
# current_dir = os.path.dirname(os.path.abspath(__file__))
# if current_dir not in sys.path:
#     sys.path.append(current_dir)
from pathlib import Path
# project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

from xiangyang.loco.common.logger import setup_logger

from phone_touch_task import PhoneTouchController, get_mode
from touch_exceptions import *

# 全局控制器实例（复用连接）
_GLOBAL_CONTROLLER: Optional[PhoneTouchController] = None
logger = setup_logger("phone_touch_interface")

def _get_system_params(interface: str = "eth0") -> dict:
    """自动检测系统状态并返回对应的参数配置"""
    try:
        # 初始化SDK以获取状态
        ChannelFactoryInitialize(0, interface)
        sport_client = LocoClient()
        sport_client.SetTimeout(3.0)
        sport_client.Init()
        
        cur_id = get_mode(sport_client.GetFsmId())
        cur_mode = get_mode(sport_client.GetFsmMode())
        
        # 释放SDK连接，避免与Controller冲突
        # sport_client.Close() # SDK没有显式Close，依赖GC或不冲突
        
        logger.info(f"🔍 接口层检测状态: ID={cur_id}, Mode={cur_mode}")
        
        # 走跑模式
        if cur_id == 801 and cur_mode is not None and cur_mode != 2:
            return {
                "expected_torso_z": -0.17,
                "measurement_error": [0.005, -0.05, 0.25],
                "wrist_pitch": -0.70,
                "torso_x_range": (0.25, 0.39),
                "torso_y_range": (0.14, 0.38)
            }
        # 常规模式
        else:
            return {
                "expected_torso_z": -0.15,
                "measurement_error": [-0.01, -0.065, 0.23],
                "wrist_pitch": -0.60,
                "torso_x_range": (0.23, 0.38),
                "torso_y_range": (0.13, 0.38)
            }
            
    except Exception as e:
        logger.warning(f"⚠️ 状态检测失败，使用默认(常规)参数: {e}")
        return {
            "expected_torso_z": -0.15,
            "measurement_error": [-0.01, -0.08, 0.24],
            "wrist_pitch": -0.55,
            "torso_x_range": (0.23, 0.38),
            "torso_y_range": (0.13, 0.38)
        }

def get_controller(interface: str = "eth0", force_reload: bool = False) -> PhoneTouchController:
    """获取或创建全局控制器实例"""
    global _GLOBAL_CONTROLLER
    
    if _GLOBAL_CONTROLLER is None or force_reload:
        params = _get_system_params(interface)
        
        _GLOBAL_CONTROLLER = PhoneTouchController(
            interface=interface,
            expected_torso_z=params["expected_torso_z"],
            torso_z_tolerance=0.05,
            measurement_error=params["measurement_error"],
            wrist_pitch=params["wrist_pitch"],
            torso_x_range=params["torso_x_range"],
            torso_y_range=params["torso_y_range"]
        )
        
        # 初始化（如果失败会抛出异常）
        _GLOBAL_CONTROLLER.initialize()
        
    return _GLOBAL_CONTROLLER

def touch_target(target_index: int, interface: str = "eth0", auto_confirm: bool = True, speak_msg: str = "出现跳闸"):
    """
    执行触碰指定目标区域的任务
    
    Args:
        target_index: 目标区域编号 (0-35)
        interface: 网络接口
        auto_confirm: 是否自动确认（跳过人工输入y/n）
        speak_msg: 任务完成时的播报内容
        
    Raises:
        TouchSystemError: 及其子类，包含所有可能的失败情况
    """
    try:
        controller = get_controller(interface)
        
        # 执行任务，confirm=False 表示不需要人工确认
        # controller.execute_task(target_index, confirm=not auto_confirm)
        controller.execute_task(target_index, confirm=not auto_confirm, speak_msg=speak_msg)
        
    except Exception as e:
        # 确保异常被传播，可以在这里添加统一的日志记录
        logger.error(f"🚨 接口层捕获异常: {e}")
        raise

def shutdown():
    """清理资源"""
    global _GLOBAL_CONTROLLER
    if _GLOBAL_CONTROLLER:
        _GLOBAL_CONTROLLER.shutdown()
        _GLOBAL_CONTROLLER = None

if __name__ == "__main__":
    # 测试接口
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=int)
    args = parser.parse_args()
    
    try:
        touch_target(args.target)
        logger.info("✅ 接口调用成功")
    except Exception as e:
        logger.error(f"❌ 接口调用失败: {type(e).__name__}: {e}")
        sys.exit(1)
