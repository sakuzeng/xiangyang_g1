#!/usr/bin/env python3
"""
G1迎宾演示 V3 (重构版)
功能：
- 协调 GreetingSkill 和 AdvancedLocomotionController
- 执行业务流程
"""
import sys
import os
import time
from pathlib import Path
# 添加路径以便导入
# current_dir = os.path.dirname(__file__)
# project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))
project_root = str(Path(__file__).resolve().parents[3])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from xiangyang.loco.common import AdvancedLocomotionController
from xiangyang.loco.skills.greeting_skill import GreetingSkill
from xiangyang.loco.common import WakeControl, TTSClient

def main():
    # === 配置 ===
    VOICE_TEXT = "尊敬的各位领导，大家好，我是监控机器人小安，欢迎莅临江南集控站指导工作。"
    INTERFACE = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    TTS_SOURCE = "greeting_demo"
    
    # === 初始化 ===
    print("🚀 启动迎宾演示程序...")
    ChannelFactoryInitialize(0, INTERFACE)
    
    # 实例化各模块
    loco = AdvancedLocomotionController(interface=INTERFACE)
    greeter = GreetingSkill(interface=INTERFACE, arm_side="right")
    
    # 统一初始化
    if not loco.initialize():
        sys.exit(1)
        
    # 显式初始化 GreetingSkill，确保姿态文件加载和连接成功
    if not greeter.initialize():
        sys.exit(1)

    try:
        # 申请 TTS 独占
        print(f"🔒 申请 TTS 独占 ({TTS_SOURCE})...")
        if not TTSClient.set_exclusive_mode(active=True, allowed_source=TTS_SOURCE):
            print("❌ 无法获取 TTS 独占权，程序退出")
            return

        # 使用 WakeControl 上下文管理器在整个演示过程中暂停唤醒
        with WakeControl(source=TTS_SOURCE):
            print("\n" + "="*50)
            print("🎬 开始业务流程")
            print("="*50)

            # [步骤1] 打招呼
            print("\n[1/4] 执行打招呼")
            # 传入 tts_source 以便 GreetingSkill 使用正确的源发送语音
            if not greeter.perform(VOICE_TEXT, tts_source=TTS_SOURCE):
                return

            # [步骤2] 左转
            print("\n[2/4] 向左转 90度")
            loco.turn_angle(90, "left")

            # [步骤3] 前进
            print("\n[3/4] 前进 0.9米")
            loco.move_forward_precise(0.9) # 保持与 v2 参数一致

            # [步骤4] 右转
            print("\n[4/4] 向右转 90度")
            loco.turn_angle(90, "right")

            print("\n✅ 演示结束")

    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    finally:
        # 清理资源
        loco.cleanup()
        greeter.stop()
        
        # 释放 TTS 独占
        print(f"🔓 释放 TTS 独占 ({TTS_SOURCE})...")
        TTSClient.set_exclusive_mode(active=False, allowed_source=TTS_SOURCE)

if __name__ == "__main__":
    main()