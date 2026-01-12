#!/usr/bin/env python3
"""
emergency_call_demo.py
======================

人机交互演示：
1. 播报异常提示
2. 调用 ASR 服务录音识别（使用 VAD 模式）
3. 识别意图 (是否拨打电话)
4. 执行拨号动作
"""

import sys
import os
import time
import requests

# 添加项目根目录到路径以便导入 common 模块
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root not in sys.path:
    sys.path.append(project_root)

try:
    from xiangyang.loco.common.tts_client import TTSClient
    from xiangyang.loco.common.asr_client import ASRClient
except ImportError as e:
    print(f"❌ 无法导入通用客户端模块: {e}")
    sys.exit(1)

# 导入拨号接口
try:
    from xiangyang.loco.ik.phone_touch_interface import touch_target, TouchSystemError, shutdown
except ImportError:
    print("❌ 无法导入 phone_touch_interface，请检查路径")
    sys.exit(1)

class EmergencyDemo:
    """紧急呼叫演示"""
    
    def __init__(self, interface_name="eth0"):
        self.interface_name = interface_name
        
    def run(self):
        try:
            # 1. 播报提示
            TTSClient.speak("是否需要拨打对应变电站电话", wait=True)
            
            # 2. 🆕 调用 ASR 服务录音识别（VAD 模式）
            print("🤔 录音4s")
            text = ASRClient.recognize_live(
                duration=4.0,
                wait_time=4.0,
                max_duration=4.0,
                silence_timeout=2.0
            )
            print(f"📝 识别结果: [{text}]")
            
            if not text:
                print("⚠️ 未检测到语音或识别失败")
                TTSClient.speak("未检测到语音，操作取消", wait=True)
                return

            # 3. 关键词匹配
            keywords = ["需要", "是", "拨打", "确认", "好的", "对", "许可"]
            confirmed = any(k in text for k in keywords)
            
            if confirmed:
                print("✅ 用户确认拨打电话")
                TTSClient.speak("正在为您拨通，请稍候", wait=False)
                
                # 4. 执行拨号
                try:
                    touch_target(31, auto_confirm=True, speak_msg="出现跳闸")
                except Exception as e:
                    print(f"❌ 拨号任务失败: {e}")
                    TTSClient.speak("拨号失败，请检查设备状态", wait=True)
            else:
                print("❌ 用户未确认或意图不明")
                TTSClient.speak("好的，已取消操作", wait=True)
                
        finally:
            print("🔧 正在释放机械臂控制权...")
            shutdown()
            print("👋 程序已结束")

if __name__ == "__main__":
    demo = EmergencyDemo(interface_name="eth0")
    try:
        demo.run()
    except KeyboardInterrupt:
        print("\n🛑 用户中断")