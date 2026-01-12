#!/usr/bin/env python3
"""
emergency_call_service.py
=========================
紧急呼叫服务（移除音频采集代码）
"""

import sys
import os
import uuid
import time
from typing import Dict, List, Optional
from datetime import datetime
import uvicorn
import threading
import queue
import requests
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# 添加当前目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# 导入依赖模块
try:
    from emergency_call_demo import TTSClient, ASRClient  # 🆕 只导入客户端
    from phone_touch_interface import touch_target, TouchSystemError, shutdown
except ImportError as e:
    print(f"❌ 导入模块失败: {e}")
    sys.exit(1)

# 全局任务队列
task_queue = queue.Queue()
tasks_store: Dict[str, dict] = {}

# 配置
TTS_CONTROL_URL = "http://192.168.77.103:28001/control/exclusive_mode"

def parse_exception_causes(exc: Exception) -> List[str]:
    """解析异常原因"""
    doc = exc.__doc__
    if not doc:
        return []
    
    causes = []
    lines = doc.split('\n')
    for line in lines:
        if '•' in line:
            parts = line.split('•')
            for part in parts:
                cleaned = part.strip()
                if cleaned:
                    causes.append(cleaned)
    return causes

def worker():
    """后台工作线程：串行处理任务"""
    print("👷 任务处理线程已启动，等待任务...")
    while True:
        try:
            item = task_queue.get()
            if item is None:
                break
            
            task_id, speak_msg, target_index = item
            print(f"🔄 开始处理任务 [{task_id}]: 内容='{speak_msg}', 目标={target_index}")
            
            if task_id in tasks_store:
                tasks_store[task_id]["status"] = "processing"
                tasks_store[task_id]["started_at"] = datetime.now().isoformat()
            
            try:
                execute_emergency_task(speak_msg, target_index)
                
                if task_id in tasks_store:
                    tasks_store[task_id]["status"] = "completed"
                    tasks_store[task_id]["completed_at"] = datetime.now().isoformat()
                    
            except Exception as e:
                error_type = type(e).__name__
                error_msg = str(e)
                causes = parse_exception_causes(e)
                
                print(f"❌ 任务 [{task_id}] 失败: {error_type} - {error_msg}")
                
                if task_id in tasks_store:
                    tasks_store[task_id].update({
                        "status": "failed",
                        "error": error_msg,
                        "error_type": error_type,
                        "possible_causes": causes,
                        "completed_at": datetime.now().isoformat()
                    })
                
                TTSClient.speak("操作执行失败，请检查设备", wait=True)
            
            task_queue.task_done()
            print(f"🏁 任务结束，队列剩余任务数: {task_queue.qsize()}")
            
        except Exception as e:
            print(f"❌ Worker 线程异常: {e}")

def set_tts_exclusive(active: bool, max_wait_seconds=3):
    """控制语音服务的独占模式"""
    try:
        payload = {
            "active": active,
            "allowed_source": "emergency_call"
        }
        
        if active:
            start_time = time.time()
            attempt = 0
            
            while time.time() - start_time < max_wait_seconds:
                attempt += 1
                response = requests.post(TTS_CONTROL_URL, json=payload, timeout=2.0)
                
                if response.status_code == 200:
                    data = response.json()
                    is_granted = data.get("is_granted", False)
                    
                    if is_granted:
                        print(f"✅ [emergency_call] 成功获得TTS独占权 (第{attempt}次尝试)")
                        return True
                    else:
                        current_source = data.get("current_source")
                        print(f"⚠️ [emergency_call] 等待独占权... (当前持有者: {current_source})")
                        time.sleep(0.3)
                else:
                    print(f"⚠️ 设置TTS独占模式请求失败: HTTP {response.status_code}")
                    return False
            
            print(f"❌ [emergency_call] 获取独占权超时 ({max_wait_seconds}秒)")
            return False
        else:
            response = requests.post(TTS_CONTROL_URL, json=payload, timeout=2.0)
            if response.status_code == 200:
                data = response.json()
                if data.get("is_granted", False):
                    print(f"🔓 [emergency_call] TTS独占模式已释放")
                    return True
                else:
                    print(f"⚠️ [emergency_call] 释放独占模式失败: {data.get('message', '未知错误')}")
                    return False
            return False
            
    except Exception as e:
        print(f"⚠️ 设置TTS独占模式异常: {e}")
        return False

def execute_emergency_task(speak_msg: str, target_index: int):
    """后台执行任务逻辑"""
    print(f"\n📨 收到请求: 目标={target_index}, 内容='{speak_msg}'")
    
    # 1. 获取独占模式
    if not set_tts_exclusive(True, max_wait_seconds=3):
        error_msg = "无法获取TTS独占权 (超时或服务异常)"
        print(f"❌ {error_msg}")
        TTSClient.speak("系统繁忙，请稍后再试", wait=True)
        raise TouchSystemError(error_msg)
    
    # 2. 切换 Source
    original_source = TTSClient.DEFAULT_SOURCE
    TTSClient.DEFAULT_SOURCE = "emergency_call"

    try:
        # 播报询问提示
        prompt_msg = "是否需要拨打对应变电站电话"
        TTSClient.speak(prompt_msg, wait=True)
        
        # 🆕 调用 ASR 服务录音识别（VAD 模式，自动检测）
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
            TTSClient.speak("未检测到语音,操作取消", wait=True)
            raise TouchSystemError("语音交互超时或未检测到语音")
        
        # 关键词匹配
        keywords = ["需要", "是", "拨打", "确认", "好的", "对", "许可"]
        confirmed = any(k in text for k in keywords)
        
        if confirmed:
            print("✅ 用户确认拨打电话")
            TTSClient.speak("正在为您拨通，请稍候", wait=False)
            touch_target(target_index, auto_confirm=True, speak_msg=speak_msg)
            print(f"✅ 任务执行完成: {speak_msg}")
        else:
            print("❌ 用户未确认或意图不明")
            TTSClient.speak("好的，已取消操作", wait=True)
        
    except Exception as e:
        raise e
    finally:
        TTSClient.DEFAULT_SOURCE = original_source
        set_tts_exclusive(False)
        print("🔧 释放机械臂控制权...")
        shutdown()

# ================= FastAPI 应用 =================
@asynccontextmanager
async def lifespan(app: FastAPI):
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    yield

app = FastAPI(title="Emergency Call Service", description="机械臂紧急呼叫服务", lifespan=lifespan)

class CallRequest(BaseModel):
    speak_msg: str
    target_index: int

class TaskResponse(BaseModel):
    task_id: str
    status: str
    message: str
    queue_position: int

class TaskStatus(BaseModel):
    task_id: str
    status: str
    created_at: str
    started_at: Optional[str] = None
    completed_at: Optional[str] = None
    error: Optional[str] = None
    error_type: Optional[str] = None
    possible_causes: Optional[List[str]] = None
    request_data: Optional[dict] = None

@app.get("/emergency_call")
def emergency_call_info():
    return {
        "info": "此接口用于触发紧急呼叫，请使用 POST 方法",
        "usage": "POST /emergency_call",
        "example_body": {
            "speak_msg": "出现跳闸",
            "target_index": 31
        }
    }

@app.get("/emergency_call/status/{task_id}", response_model=TaskStatus)
def get_task_status(task_id: str):
    if task_id not in tasks_store:
        raise HTTPException(status_code=404, detail="Task not found")
    return tasks_store[task_id]

@app.post("/emergency_call", response_model=TaskResponse)
async def trigger_emergency_call(request: CallRequest):
    if request.target_index < 0 or request.target_index > 35:
        raise HTTPException(status_code=400, detail="Target index must be between 0 and 35")
    
    task_id = str(uuid.uuid4())
    position = task_queue.qsize()
    
    tasks_store[task_id] = {
        "task_id": task_id,
        "status": "queued",
        "created_at": datetime.now().isoformat(),
        "request_data": request.dict()
    }
    
    task_queue.put((task_id, request.speak_msg, request.target_index))
    
    logger_msg = f"任务 [{task_id}] 已加入队列，前方排队数: {position}"
    print(f"📥 {logger_msg}")
    
    return {
        "task_id": task_id,
        "status": "queued",
        "message": logger_msg,
        "queue_position": position
    }

@app.get("/health")
def health_check():
    return {"status": "ok", "service": "emergency_call_service"}

if __name__ == "__main__":
    print("🚀 启动紧急呼叫服务...")
    print("📡 监听地址: http://0.0.0.0:9000")
    uvicorn.run(app, host="0.0.0.0", port=9000)
