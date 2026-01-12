#!/usr/bin/env python3
"""
emergency_call_demo.py
======================

人机交互演示：
1. 播报异常提示
2. 监听用户语音 (6秒)
3. 识别意图 (是否拨打电话)
4. 执行拨号动作 (Touch Interface)
"""

import sys
import os
import time
import socket
import threading
import tempfile
import numpy as np
import requests

# 添加 audio 目录到路径以导入依赖
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../audio')))

try:
    from audio_record import AudioRecorder, CHANNELS, SAMPLE_WIDTH, FRAME_RATE
except ImportError:
    print("❌ 无法导入 audio_record，请检查路径")
    sys.exit(1)

# 导入拨号接口
try:
    from phone_touch_interface import touch_target, TouchSystemError, shutdown
except ImportError:
    print("❌ 无法导入 phone_touch_interface，请检查路径")
    sys.exit(1)

# 配置 (与 integrated_wake_recorder.py 保持一致)
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"
TTS_MONITOR_URL = "http://192.168.77.103:28001/monitor"
ASR_SERVER_URL = "http://localhost:8003/asr"

class TTSClient:
    """HTTP TTS 客户端 (简化版)"""
    DEFAULT_SOURCE = "integrated"
    
    @staticmethod
    def speak(text, volume=100, wait=True, source=None):
        """发送TTS请求并可选等待播放完成"""
        if not text:
            return
        
        # 使用传入的 source，否则使用类属性中的默认值
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
    """HTTP ASR 客户端 (简化版)"""
    
    @staticmethod
    def recognize(audio_data):
        """调用远程 ASR 服务识别音频"""
        if audio_data is None or len(audio_data) == 0:
            return ""
        
        try:
            # 转换为WAV
            import wave
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_file:
                tmp_path = tmp_file.name
                with wave.open(tmp_path, 'wb') as wav_file:
                    wav_file.setnchannels(CHANNELS)
                    wav_file.setsampwidth(SAMPLE_WIDTH)
                    wav_file.setframerate(FRAME_RATE)
                    wav_file.writeframes(audio_data.tobytes())
            
            # 上传识别
            with open(tmp_path, 'rb') as f:
                files = {'file': (f'audio.wav', f, 'audio/wav')}
                response = requests.post(ASR_SERVER_URL, files=files, timeout=10.0)
            
            os.remove(tmp_path)
            
            if response.status_code == 200:
                result = response.json()
                return result.get('text', '').strip()
            return ""
            
        except Exception as e:
            print(f"❌ ASR失败: {e}")
            return ""

class EmergencyDemo(AudioRecorder):
    def __init__(self, interface_name="eth0"):
        super().__init__(interface_name)
        self.running = True
        # 初始化音频接收
        self.setup_audio_receiver()
        
    def listen_for_seconds(self, duration=6.0):
        """监听指定时长的音频"""
        print(f"\n👂 开始监听 ({duration}秒)...")
        buffer = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            try:
                data, _ = self.socket.recvfrom(2048)
                audio_np = np.frombuffer(data, dtype=np.int16)
                buffer.extend(audio_np)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ 录音错误: {e}")
                break
                
        print("⏹️ 监听结束")
        return np.array(buffer)

    def run(self):
        try:
            # 1. 播报提示
            TTSClient.speak("出现异常，是否需要拨打对应变电站电话", wait=True)
            
            # 2. 监听回复
            audio_data = self.listen_for_seconds(6.0)
            
            if len(audio_data) == 0:
                print("⚠️ 未采集到音频数据")
                TTSClient.speak("未检测到语音，操作取消", wait=True)
                return

            # 3. 识别意图
            print("🤔 正在识别...")
            text = ASRClient.recognize(audio_data)
            print(f"📝 识别结果: [{text}]")
            
            # 4. 关键词匹配
            keywords = ["需要", "是", "拨打", "确认", "好的"]
            confirmed = any(k in text for k in keywords)
            
            if confirmed:
                print("✅ 用户确认拨打电话")
                TTSClient.speak("正在为您拨通，请稍候", wait=False) # 不等待，边说边做
                
                # 5. 执行拨号
                try:
                    # 调用接口，auto_confirm=True 手动确认
                    touch_target(31, auto_confirm=True, speak_msg="出现跳闸")
                except Exception as e:
                    print(f"❌ 拨号任务失败: {e}")
                    TTSClient.speak("拨号失败，请检查设备状态", wait=True)
            else:
                print("❌ 用户未确认或意图不明")
                TTSClient.speak("好的，已取消操作", wait=True)
                
        finally:
            self.socket.close()
            print("🔧 正在释放机械臂控制权...")
            shutdown()
            print("👋 程序已结束")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        interface = "eth0"
    else:
        interface = sys.argv[1]
        
    demo = EmergencyDemo(interface)
    try:
        demo.run()
    except KeyboardInterrupt:
        print("\n🛑 用户中断")