import sys
import time
import signal
import threading
import socket
import json
import requests
import numpy as np
import tempfile
import os
import uuid
from collections import deque

from audio_record import AudioRecorder, CHANNELS, SAMPLE_WIDTH, FRAME_RATE
from wake_word_detector import WakeWordDetector 

# 配置
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"
TTS_MONITOR_URL = "http://192.168.77.103:28001/monitor"
ASR_SERVER_URL = "http://localhost:8003/asr"
AGENT_SERVER_URL = "http://192.168.77.102:8602/v1/chat/completions"
WAKE_WORD = "小安"

class TTSClient:
    """HTTP TTS 客户端 - 流式播放 + 等待完成"""
    
    @staticmethod
    def speak(text, volume=80, wait=True, source="integrated"):
        """发送TTS请求并可选等待播放完成"""
        if not text:
            return
        
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
            
            if result.get('msg') == 'ignored_filtered':
                print(f"⚠️ TTS被过滤")
                return
            
            data = result.get('data')
            if not data or not isinstance(data, dict):
                print(f"⚠️ TTS响应异常")
                return
            
            task_id = data.get('task_id')
            
            if wait and task_id:
                TTSClient._wait_for_completion(task_id)
                
        except requests.exceptions.RequestException as e:
            print(f"❌ TTS请求失败: {e}")
        except Exception as e:
            print(f"❌ TTS失败: {e}")

    @staticmethod
    def _wait_for_completion(task_id, timeout=30):
        """轮询监控接口，等待任务完成（终极优化版）"""
        start_time = time.time()
        check_interval = 0.05  # 🔑 缩短轮询间隔到 50ms
        
        task_started = False
        consecutive_empty_checks = 0  # 🔑 连续空闲检查次数
        
        while time.time() - start_time < timeout:
            try:
                response = requests.get(TTS_MONITOR_URL, timeout=2.0)
                
                if response.status_code == 200:
                    data = response.json()
                    active_task = data.get('active_task')
                    queue_length = data.get('queue_length', 0)
                    
                    # 检查任务是否正在播放
                    if active_task:
                        current_id = active_task.get('id')
                        if current_id == task_id:
                            task_started = True
                            consecutive_empty_checks = 0  # 重置计数器
                            time.sleep(check_interval)
                            continue
                    
                    # 🔑 修复: 任务开始后，需要连续3次检查都为空才认为完成
                    if task_started:
                        if not active_task and queue_length == 0:
                            consecutive_empty_checks += 1
                            
                            # 🔑 关键: 连续3次空闲检查才返回
                            if consecutive_empty_checks >= 3:
                                # 🔑 额外延迟确保音频完全播放完毕
                                time.sleep(0.2)
                                return True
                        else:
                            consecutive_empty_checks = 0
                    
            except:
                pass
                
            time.sleep(check_interval)
        
        return False

class ASRClient:
    """HTTP ASR 客户端"""
    
    @staticmethod
    def recognize(audio_data, use_itn=False, verbose=False):
        """
        调用远程 ASR 服务识别音频
        :param verbose: 是否输出详细日志
        """
        if audio_data is None or len(audio_data) == 0:
            return ""
        
        try:
            # 将音频数据转换为 WAV 临时文件
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_file:
                tmp_path = tmp_file.name
                
                import wave
                with wave.open(tmp_path, 'wb') as wav_file:
                    wav_file.setnchannels(CHANNELS)
                    wav_file.setsampwidth(SAMPLE_WIDTH)
                    wav_file.setframerate(FRAME_RATE)
                    wav_file.writeframes(audio_data.tobytes())
            
            # 上传到 ASR 服务
            with open(tmp_path, 'rb') as f:
                files = {'file': (f'audio_{int(time.time())}.wav', f, 'audio/wav')}
                response = requests.post(ASR_SERVER_URL, files=files, timeout=10.0)
            
            # 清理临时文件
            try:
                os.remove(tmp_path)
            except:
                pass
            
            # 解析结果
            if response.status_code == 200:
                result = response.json()
                text = result.get('text', '').strip()
                return text
            else:
                return ""
                
        except Exception as e:
            if verbose:
                print(f"❌ ASR失败: {e}")
            return ""

class AgentClient:
    """大模型对话 Agent 客户端"""
    
    def __init__(self):
        self.api_url = AGENT_SERVER_URL
        self.session_id = str(uuid.uuid4())
        self.memory_data = None # 记忆数据，初始为空
        
    def chat(self, query):
        """发送对话请求"""
        if not query:
            return "请再说一次"
            
        payload = {
            "session_id": self.session_id,
            "request_id": str(uuid.uuid4()),
            "query": query,
            "memory_data": self.memory_data
        }
        
        try:
            print(f"🤖 请求Agent: {query}")
            response = requests.post(self.api_url, json=payload, timeout=15.0)
            
            if response.status_code == 200:
                data = response.json()
                
                # 提取回复
                reply_text = data.get("response", "")
                if not reply_text:
                    return "Agent回复为空"
                    
                # 更新状态（session_id 和 memory）
                if "session_id" in data:
                    self.session_id = data["session_id"]
                if "memory" in data:
                    self.memory_data = data["memory"]
                    
                return reply_text
            else:
                print(f"❌ Agent API错误: {response.status_code}")
                return "大脑连接出错了"
                
        except requests.exceptions.Timeout:
            print("❌ Agent请求超时")
            return "思考超时了"
        except Exception as e:
            print(f"❌ Agent请求失败: {e}")
            return "网络开小差了"

class RobotInteractionSystem(AudioRecorder):
    """精简版人机交互系统 - 使用远程 ASR"""
    
    def __init__(self, interface_name="eth0"):
        super().__init__(interface_name)
        
        print("🚀 初始化交互系统...")
        
        # 测试 ASR 服务连通性（静默）
        try:
            response = requests.get(ASR_SERVER_URL.replace('/asr', '/'), timeout=3.0)
            if response.status_code == 200:
                print(f"✅ ASR服务连接成功")
        except:
            print(f"⚠️ ASR服务连接失败")

        # 🔑 静默初始化唤醒检测器
        self.wake_detector = WakeWordDetector(
            target_wake_word=WAKE_WORD, 
            confidence_threshold=0.6,
            verbose=False  # 关键：不输出初始化日志
        )
        
        # 初始化 Agent 客户端
        self.agent_client = AgentClient()
        
        # 状态控制
        self.is_running = True
        self.audio_running = True
        self.is_speaking = False  # 🤖 机器人是否正在说话
        self.mode = "WAKE_DETECTION"
        
        # 音频缓冲
        self.wake_buffer = deque(maxlen=int(3.0 * FRAME_RATE))
        self.user_speech_buffer = []
        self.user_speech_start_time = 0
        self.user_speech_timeout = 5.0
        
        # 🔑 关键修改：先设置 socket，再启动线程
        self.setup_audio_receiver()
        
        # 启动处理线程
        self.process_thread = threading.Thread(target=self._audio_processing_loop, daemon=True)
        self.process_thread.start()
        
        # 启动音频接收线程
        self.audio_thread = threading.Thread(target=self._audio_receiver_loop, daemon=True)
        self.audio_thread.start()

        # ⏳ 启动倒计时，让用户明确知道何时可以说话
        print("\n⏳ 系统准备中...")
        for i in range(3, 0, -1):
            print(f"   {i}...", end="\r")
            time.sleep(0.5)
        print("   🚀 请说话!   \n")

    def _audio_receiver_loop(self):
        """音频接收线程（静默）"""
        while self.audio_running:
            try:
                data, addr = self.socket.recvfrom(2048)
                self.process_audio_frame(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.audio_running:
                    print(f"❌ 音频接收错误: {e}")
                break

    def process_audio_frame(self, audio_data):
        """接收UDP音频数据（静默）"""
        # 🤖 如果机器人正在说话，丢弃麦克风数据（防止听到自己）
        if self.is_speaking:
            return

        try:
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            
            if self.mode == "WAKE_DETECTION":
                self.wake_buffer.extend(audio_np)
            elif self.mode == "USER_RECORDING":
                self.user_speech_buffer.extend(audio_np)
                
        except:
            pass

    def speak(self, text):
        """封装的说话方法，说话期间暂停录音"""
        if not text:
            return
            
        self.is_speaking = True
        try:
            # 清空缓冲区，防止之前的残留
            self.wake_buffer.clear()
            # self.user_speech_buffer = [] # list 没有 clear? Python3 应该有，或者重新赋值
            self.user_speech_buffer.clear()
            
            TTSClient.speak(text, wait=True)
        finally:
            self.is_speaking = False
            # 说话结束后再次清空，确保干净
            self.wake_buffer.clear()
            self.user_speech_buffer.clear()

    def _audio_processing_loop(self):
        """主处理循环"""
        print(f"👂 监听唤醒词: {WAKE_WORD}\n")
        
        while self.is_running:
            if self.mode == "WAKE_DETECTION":
                self._do_wake_detection()
                # 🔑 增加间隔到 0.3s，避免检测太频繁把一句话切碎
                time.sleep(0.3)
                
            elif self.mode == "USER_RECORDING":
                self._check_recording_timeout()
                time.sleep(0.05)
            
    def _do_wake_detection(self):
        """执行唤醒检测"""
        # 缓冲区数据不足时不检测 (从0.2秒恢复到0.5秒，保证有足够数据)
        if len(self.wake_buffer) < int(0.5 * FRAME_RATE):
            return

        # 🔑 优化：计算最近 1.0 秒的能量 (从0.5s增加到1.0s)
        # 既能避免长静音稀释，又能包含完整词语
        check_len = int(1.0 * FRAME_RATE)
        curr_audio = np.array(list(self.wake_buffer))
        
        if len(curr_audio) > check_len:
            recent_audio = curr_audio[-check_len:]
        else:
            recent_audio = curr_audio
            
        energy = np.sqrt(np.mean(recent_audio.astype(np.float32) ** 2))
        
        # 调试：始终打印能量值
        if energy > 10:
            print(f"\r🔊 能量: {energy:.1f}   ", end="", flush=True)
        
        # 阈值保持 200
        if energy < 200:
            return
            
        print(f"\n⚡ 能量触发 ({energy:.1f})，正在请求ASR识别...")

        # ASR 识别 (发送最近 1.5 秒的数据，确保包含完整唤醒词)
        # 即使能量只计算了1秒，识别时多发一点数据更保险
        recognize_len = int(1.5 * FRAME_RATE)
        if len(curr_audio) > recognize_len:
            audio_to_recognize = curr_audio[-recognize_len:]
        else:
            audio_to_recognize = curr_audio

        # ASR 识别 (开启详细模式以便调试错误)
        text = ASRClient.recognize(audio_to_recognize, use_itn=False, verbose=True)
        
        if not text:
            # 如果是高能量但识别为空，可能是网络问题或无法识别
            # print("⚠️ ASR返回为空")
            return
            
        print(f"👂 ASR识别结果: [{text}]")
        if not text:
            return

        # 模糊匹配唤醒词
        is_wake, conf, match = self.wake_detector.detect_wake_word(text)
        
        # 🔑 只有唤醒成功时才输出日志
        if is_wake:
            print(f"✨ 唤醒成功! (匹配: {match}, 置信度: {conf:.2f})")
            
            # 播放回应并等待完成 (使用封装的 speak 方法)
            self.speak("我在，请吩咐。")
            
            # TTS 播放完成后才切换状态
            self._switch_to_recording()

    def _switch_to_recording(self):
        """切换到用户指令录制模式"""
        self.mode = "USER_RECORDING"
        self.user_speech_buffer = []
        self.user_speech_start_time = time.time()
        print(f"🎤 录制中... (最大 {self.user_speech_timeout}秒)\n")

    def _check_recording_timeout(self):
        """检查录音是否超时或结束"""
        elapsed = time.time() - self.user_speech_start_time
        
        if elapsed > self.user_speech_timeout:
            print("⏹️ 录音结束，开始处理...\n")
            print("🚫 系统忙：正在思考和回答 (此时无法唤醒)")
            self._process_user_intent()
            
            # 处理完后切回唤醒模式
            self.mode = "WAKE_DETECTION"
            self.wake_detector.reset_cooldown()
            
            # 🔑 关键优化：切回唤醒模式时，不完全清空缓冲区
            # 而是保留最后 1.0 秒的音频，防止用户抢话导致的漏检
            keep_samples = int(1.0 * FRAME_RATE)
            if len(self.user_speech_buffer) > keep_samples:
                # 从用户录音缓冲的末尾提取数据填入唤醒缓冲
                recent_audio = self.user_speech_buffer[-keep_samples:]
                self.wake_buffer.extend(recent_audio)
            else:
                self.wake_buffer.clear()
                
            print(f"👂 监听恢复: {WAKE_WORD}\n")

    def _process_user_intent(self):
        """处理用户指令"""
        if not self.user_speech_buffer:
            return

        audio_data = np.array(self.user_speech_buffer)
        
        # 识别用户说的话（详细模式）
        text = ASRClient.recognize(audio_data, use_itn=True, verbose=True)
        
        if text:
            print(f"📝 识别: {text}")
            # 简单的确认（可选，如果Agent响应快可以去掉，或者保留作为填补空白）
            # TTSClient.speak(f"收到,您说的是:{text}", wait=True)
            
            # 调用 Agent 获取回复
            print("🤔 思考中...")
            agent_response = self.agent_client.chat(text)
            print(f"🤖 回复: {agent_response}")
            
            # 播放 Agent 回复
            self.speak(agent_response)
            
        else:
            print(f"📝 识别: (空)")
            self.speak("抱歉,我没听清。")
        
        print()  # 空行分隔

    def cleanup(self):
        """资源清理"""
        self.is_running = False
        self.audio_running = False
        
        if hasattr(self, 'process_thread') and self.process_thread.is_alive():
            self.process_thread.join(timeout=2)
        if hasattr(self, 'audio_thread') and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2)
            
        super().cleanup()

def main():
    def signal_handler(sig, frame):
        print("\n🛑 退出")
        if 'system' in locals():
            system.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)

    if len(sys.argv) < 2:
        interface = "eth0"
    else:
        interface = sys.argv[1]

    system = RobotInteractionSystem(interface)
    
    try:
        # 🔑 关键修改：移除这里的 setup_audio_receiver() 调用
        # system.setup_audio_receiver()  # 已在 __init__ 中调用
        
        print("🎯 系统运行中 (Ctrl+C 退出)\n")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n接收到退出信号")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        system.cleanup()

if __name__ == "__main__":
    main()
