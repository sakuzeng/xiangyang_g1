import sys
import socket
import struct
import threading
import io
import wave
import asyncio
import netifaces
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI, Response, HTTPException
from pydantic import BaseModel, Field
from typing import Optional

# --- 配置参数 ---
SERVER_PORT = 28000
MULTICAST_GROUP = "239.168.123.161"
MULTICAST_PORT = 5555
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit
FRAME_RATE = 16000
DEFAULT_INTERFACE = "eth0"

# --- 核心音频管理类 (保持不变) ---
class AudioStreamManager:
    def __init__(self, interface_name):
        self.interface_name = interface_name
        self.socket = None
        self.running = False
        self.recording_active = False
        self.audio_buffer = [] 
        self.thread = None
        self.thread_lock = threading.Lock()
        self.request_lock = asyncio.Lock()

    def get_local_ip_for_multicast(self):
        """获取192.168.123.x网段的IP地址"""
        for interface in netifaces.interfaces():
            try:
                addresses = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addresses:
                    for addr_info in addresses[netifaces.AF_INET]:
                        ip = addr_info['addr']
                        if ip.startswith('192.168.123.'):
                            return ip
            except:
                continue
        return None

    def setup_socket(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('', MULTICAST_PORT))
            
            local_ip = self.get_local_ip_for_multicast()
            if local_ip is None:
                print(f"⚠️ 未找到192.168.123.x网段，尝试使用接口 {self.interface_name} IP")
                try:
                    local_ip = netifaces.ifaddresses(self.interface_name)[netifaces.AF_INET][0]['addr']
                except:
                    local_ip = '0.0.0.0'

            print(f"🔌 绑定本地 IP: {local_ip}")

            mreq = struct.pack("4s4s",
                               socket.inet_aton(MULTICAST_GROUP),
                               socket.inet_aton(local_ip))
            self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self.socket.settimeout(1.0)
            print(f"✅ 音频监听器就绪: {MULTICAST_GROUP}:{MULTICAST_PORT}")
            return True
        except Exception as e:
            print(f"❌ Socket设置失败: {e}")
            return False

    def listener_task(self):
        print("👂 后台音频监听线程已启动")
        while self.running:
            try:
                data, _ = self.socket.recvfrom(2048)
                if self.recording_active:
                    with self.thread_lock:
                        self.audio_buffer.append(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"❌ 接收异常: {e}")
                break
        print("🛑 音频监听线程已停止")

    def start(self):
        if self.setup_socket():
            self.running = True
            self.thread = threading.Thread(target=self.listener_task, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()
        if self.thread:
            self.thread.join(timeout=2)

    def clear_buffer(self):
        with self.thread_lock:
            self.audio_buffer = []

    def get_wav_bytes(self):
        with self.thread_lock:
            if not self.audio_buffer:
                return None
            raw_data = b''.join(self.audio_buffer)
        
        wav_io = io.BytesIO()
        with wave.open(wav_io, 'wb') as wav_file:
            wav_file.setnchannels(CHANNELS)
            wav_file.setsampwidth(SAMPLE_WIDTH)
            wav_file.setframerate(FRAME_RATE)
            wav_file.writeframes(raw_data)
        
        wav_io.seek(0)
        return wav_io

# --- 全局管理 ---

audio_manager = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global audio_manager
    # 默认尝试 eth0，因为这是 Unitree 机器人的通常配置
    interface_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_INTERFACE
    
    audio_manager = AudioStreamManager(interface_name)
    audio_manager.start()
    yield
    print("正在清理资源...")
    audio_manager.stop()

app = FastAPI(title="Unitree Audio Recorder (POST)", lifespan=lifespan)

# --- 【关键修改】定义 POST 请求的数据模型 ---
class RecordConfig(BaseModel):
    duration: float = Field(..., gt=0, le=60, description="录音时长(秒)，最长60秒")
    filename_prefix: Optional[str] = Field("record", description="下载文件的文件名前缀")

# --- 【关键修改】改为 POST 接口 ---
@app.post("/record")
async def record_audio(config: RecordConfig):
    """
    POST 请求录音接口。
    接收 JSON: {"duration": 5, "filename_prefix": "my_test"}
    返回: WAV 文件流
    """
    global audio_manager
    
    if not audio_manager or not audio_manager.running:
        raise HTTPException(status_code=503, detail="Audio service not running")

    # 使用 config.duration 获取参数
    print(f"📥 收到录音请求: {config.duration}s, 前缀: {config.filename_prefix}")

    async with audio_manager.request_lock:
        try:
            audio_manager.clear_buffer()
            audio_manager.recording_active = True
            await asyncio.sleep(config.duration)
        finally:
            audio_manager.recording_active = False

        wav_io = audio_manager.get_wav_bytes()
    
    if wav_io is None:
        raise HTTPException(status_code=500, detail="No audio captured")
    
    # 生成文件名
    timestamp = int(asyncio.get_event_loop().time())
    filename = f"{config.filename_prefix}_{timestamp}.wav"
    
    return Response(
        content=wav_io.read(),
        media_type="audio/wav",
        headers={"Content-Disposition": f"attachment; filename={filename}"}
    )

if __name__ == "__main__":
    print(f"🚀 录音服务已启动 (POST模式): http://0.0.0.0:{SERVER_PORT}")
    uvicorn.run(app, host="0.0.0.0", port=SERVER_PORT)