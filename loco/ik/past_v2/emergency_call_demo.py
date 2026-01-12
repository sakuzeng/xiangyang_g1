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

# 导入拨号接口
try:
    from phone_touch_interface import touch_target, TouchSystemError, shutdown
except ImportError:
    print("❌ 无法导入 phone_touch_interface，请检查路径")
    sys.exit(1)

# 配置
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"
TTS_MONITOR_URL = "http://192.168.77.103:28001/monitor"
ASR_SERVER_URL = "http://192.168.77.103:28003/recognize_live"

class TTSClient:
    """HTTP TTS 客户端"""
    DEFAULT_SOURCE = "emergency_call"
    
    @staticmethod
    def speak(text, volume=100, wait=True, source=None):
        """发送TTS请求并可选等待播放完成"""
        if not text:
            return
        
        if source is None:
            source = TTSClient.DEFAULT_SOURCE

        try:
            payload = {
                "speak_msg": text,
                "volume": volume,
                "source": source
            }
            headers = {"Content-Type": "application/json"}
            
            print(f"🔊 {text}")
            response = requests.post(TTS_SERVER_URL, json=payload, headers=headers, timeout=2.0)
            
            if response.status_code != 200:
                print(f"⚠️ TTS错误: {response.status_code}")
                return
            
            result = response.json()
            data = result.get('data')
            if not data or not isinstance(data, dict):
                return
            
            task_id = data.get('task_id')
            
            if wait and task_id:
                TTSClient._wait_for_completion(task_id)
                
        except Exception as e:
            print(f"❌ TTS失败: {e}")

    @staticmethod
    def _wait_for_completion(task_id, timeout=30):
        """等待任务完成"""
        start_time = time.time()
        check_interval = 0.05
        task_started = False
        consecutive_empty_checks = 0
        
        while time.time() - start_time < timeout:
            try:
                response = requests.get(TTS_MONITOR_URL, timeout=2.0)
                if response.status_code == 200:
                    data = response.json()
                    active_task = data.get('active_task')
                    queue_length = data.get('queue_length', 0)
                    
                    if active_task and active_task.get('id') == task_id:
                        task_started = True
                        consecutive_empty_checks = 0
                        time.sleep(check_interval)
                        continue
                    
                    if task_started:
                        if not active_task and queue_length == 0:
                            consecutive_empty_checks += 1
                            if consecutive_empty_checks >= 3:
                                time.sleep(0.2)
                                return True
                        else:
                            consecutive_empty_checks = 0
            except:
                pass
            time.sleep(check_interval)
        return False

class ASRClient:
    """HTTP ASR 客户端（支持固定时长和 VAD 模式）"""
    
    @staticmethod
    def recognize_live(duration=None, max_duration=10.0, silence_timeout=2.0, wait_time=None):
        """
        调用 ASR 服务进行实时录音识别
        
        Args:
            duration: 录音时长(秒)，None 表示使用 VAD 模式
            max_duration: VAD 模式的最大时长
            silence_timeout: VAD 模式的静音超时
            wait_time: 接口调用等待时间 (秒)
            
        Returns:
            识别文本
        """
        try:
            payload = {
                "duration": duration,
                "max_duration": max_duration,
                "silence_timeout": silence_timeout,
                "wait_time": wait_time
            }
            
            # 计算超时时间
            base_time = duration if duration is not None else max_duration
            if wait_time is not None:
                base_time = max(base_time, wait_time)
                
            timeout = base_time + 5.0
            
            response = requests.post(ASR_SERVER_URL, json=payload, timeout=timeout)
            
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    method = result.get("method", "unknown")
                    print(f"ℹ️ 识别模式: {method}")
                    return result.get("text", "")
                else:
                    print(f"⚠️ ASR 识别失败: {result.get('error')}")
                    return ""
            else:
                print(f"⚠️ ASR 服务请求失败: HTTP {response.status_code}")
                return ""
                
        except requests.exceptions.Timeout:
            print("❌ ASR 服务超时")
            return ""
        except requests.exceptions.ConnectionError:
            print("❌ 无法连接到 ASR 服务，请确保服务已启动")
            return ""
        except Exception as e:
            print(f"❌ ASR 调用异常: {e}")
            return ""

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