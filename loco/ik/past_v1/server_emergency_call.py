#!/usr/bin/env python3
"""
emergency_call_service.py
=========================

将紧急呼叫功能封装为网络服务接口。
提供 FastAPI 服务，监听 9000 端口。

接口:
POST /emergency_call
{
    "speak_msg": "出现跳闸",
    "target_index": 31
}

curl -X POST "http://localhost:9000/emergency_call" \
     -H "Content-Type: application/json" \
     -d '{"speak_msg": "顺安变电站运维班2测试", "target_index": 31}'

"""

import sys
import os
import uuid
from typing import Dict, List, Optional
from datetime import datetime
import uvicorn
import threading
import queue
import requests
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel

# 添加当前目录到路径，以便导入同级模块
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# 导入依赖模块
try:
    # 复用 demo 中的 TTS 客户端
    from emergency_call_demo import TTSClient, EmergencyDemo, ASRClient
    # 导入触摸接口
    from phone_touch_interface import touch_target, TouchSystemError, shutdown
except ImportError as e:
    print(f"❌ 导入模块失败: {e}")
    print(f"请确保 emergency_call_demo.py 和 phone_touch_interface.py 在同一目录下")
    sys.exit(1)

# 全局任务队列
# 存储结构: (task_id, speak_msg, target_index)
task_queue = queue.Queue()

# 全局任务状态存储
# key: task_id (str)
# value: dict
tasks_store: Dict[str, dict] = {}

def parse_exception_causes(exc: Exception) -> List[str]:
    """
    解析异常类的 docstring，提取可能的错误原因
    格式假设: Docstring 中包含 '•' 分隔的原因列表
    """
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
            # 阻塞等待任务
            item = task_queue.get()
            if item is None:
                break
            
            task_id, speak_msg, target_index = item
            print(f"🔄 开始处理任务 [{task_id}]: 内容='{speak_msg}', 目标={target_index}")
            
            # 更新状态为处理中
            if task_id in tasks_store:
                tasks_store[task_id]["status"] = "processing"
                tasks_store[task_id]["started_at"] = datetime.now().isoformat()
            
            # 执行核心逻辑
            try:
                execute_emergency_task(speak_msg, target_index)
                
                # 任务成功
                if task_id in tasks_store:
                    tasks_store[task_id]["status"] = "completed"
                    tasks_store[task_id]["completed_at"] = datetime.now().isoformat()
                    
            except Exception as e:
                # 任务失败，捕获并解析异常
                error_type = type(e).__name__
                error_msg = str(e)
                causes = parse_exception_causes(e)
                
                print(f"❌ 任务 [{task_id}] 失败: {error_type} - {error_msg}")
                if causes:
                    print(f"   可能的排查方向: {causes}")
                
                if task_id in tasks_store:
                    tasks_store[task_id].update({
                        "status": "failed",
                        "error": error_msg,
                        "error_type": error_type,
                        "possible_causes": causes,
                        "completed_at": datetime.now().isoformat()
                    })
                
                # 尝试播报错误
                TTSClient.speak("操作执行失败，请检查设备", wait=True)
            
            # 标记当前任务完成
            task_queue.task_done()
            print(f"🏁 任务结束，队列剩余任务数: {task_queue.qsize()}")
            
        except Exception as e:
            print(f"❌ Worker 线程异常: {e}")

@asynccontextmanager
async def lifespan(app: FastAPI):
    # 启动后台线程
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    yield
    # 关闭时发送退出信号（可选）
    # task_queue.put(None)

# 创建 FastAPI 应用
app = FastAPI(title="Emergency Call Service", description="机械臂紧急呼叫服务", lifespan=lifespan)

class CallRequest(BaseModel):
    speak_msg: str      # 需要播报的内容 (例如: "出现跳闸")
    target_index: int   # 拨打电话的区域 (例如: 31)

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

TTS_CONTROL_URL = "http://192.168.77.103:28001/control/exclusive_mode"

def set_tts_exclusive(active: bool):
    """控制语音服务的独占模式"""
    try:
        payload = {
            "active": active,
            "allowed_source": "emergency_call" if active else None
        }
        # 这里的端口 8001 对应 server_speak_msg.py 的端口
        requests.post(TTS_CONTROL_URL, json=payload, timeout=2.0)
        print(f"🔒 TTS独占模式已{'开启' if active else '关闭'}")
    except Exception as e:
        print(f"⚠️ 设置TTS独占模式失败: {e}")

def execute_emergency_task(speak_msg: str, target_index: int):
    """后台执行任务逻辑"""
    print(f"\n📨 收到请求: 目标={target_index}, 内容='{speak_msg}'")
    
    # 1. 开启独占模式并切换 Source
    set_tts_exclusive(True)
    original_source = TTSClient.DEFAULT_SOURCE
    TTSClient.DEFAULT_SOURCE = "emergency_call"

    try:
        # TTSClient.speak(speak_msg, wait=True)
        # 1. 播报询问提示
        # 默认询问语，也可以根据请求参数定制
        prompt_msg = "是否需要拨打对应变电站电话"
        TTSClient.speak(prompt_msg, wait=True)
        
        # 2. 初始化录音实例
        # 注意：这里会绑定 UDP 端口，必须确保 finally 中释放
        demo_instance = EmergencyDemo(interface_name="eth0")
        
        # 3. 监听回复 (5秒)
        audio_data = demo_instance.listen_for_seconds(5.0)
        
        if len(audio_data) == 0:
            print("⚠️ 未采集到音频数据")
            TTSClient.speak("未检测到语音，操作取消", wait=True)
            # 抛出异常以便记录状态
            raise TouchSystemError("语音交互超时或未检测到语音")

        # 4. 识别意图
        print("🤔 正在识别...")
        text = ASRClient.recognize(audio_data)
        print(f"📝 识别结果: [{text}]")
        
        # 5. 关键词匹配
        keywords = ["需要", "是", "拨打", "确认", "好的", "对", "许可"]
        confirmed = any(k in text for k in keywords)
        
        if confirmed:
            print("✅ 用户确认拨打电话")
            TTSClient.speak("正在为您拨通，请稍候", wait=False)
            
            # 6. 执行拨号任务
            # auto_confirm=False: 这里的 False 仅仅指 touch_target 内部不再进行控制台输入确认
            # 因为我们已经在语音层做了确认
            touch_target(target_index, auto_confirm=False, speak_msg=speak_msg)
            
            print(f"✅ 任务执行完成: {speak_msg}")
        else:
            print("❌ 用户未确认或意图不明")
            TTSClient.speak("好的，已取消操作", wait=True)
        
    except Exception as e:
        # 这里的异常会被 worker 捕获并记录到任务状态中
        raise e
    finally:
        # 还原状态
        TTSClient.DEFAULT_SOURCE = original_source
        set_tts_exclusive(False)

        # 3. 任务结束后释放控制权
        print("🔧 释放机械臂控制权...")
        shutdown()

@app.get("/emergency_call")
def emergency_call_info():
    """提供接口使用说明"""
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
    """
    查询任务执行状态
    
    返回包含详细错误信息和建议排查方向（如果失败）
    """
    if task_id not in tasks_store:
        raise HTTPException(status_code=404, detail="Task not found")
    
    return tasks_store[task_id]

@app.post("/emergency_call", response_model=TaskResponse)
async def trigger_emergency_call(request: CallRequest):
    """
    触发紧急呼叫任务
    
    - **speak_msg**: 告警播报内容
    - **target_index**: 屏幕目标区域索引
    
    返回 task_id，可用于查询后续执行状态。
    """
    # 简单的参数校验
    if request.target_index < 0 or request.target_index > 35:
        raise HTTPException(status_code=400, detail="Target index must be between 0 and 35")
    
    # 生成任务ID
    task_id = str(uuid.uuid4())
    position = task_queue.qsize()
    
    # 初始化任务状态
    tasks_store[task_id] = {
        "task_id": task_id,
        "status": "queued",
        "created_at": datetime.now().isoformat(),
        "request_data": request.dict()
    }
    
    # 将任务加入队列
    task_queue.put((task_id, request.speak_msg, request.target_index))
    
    logger_msg = f"任务 [{task_id}] 已加入队列，前方排队数: {position}"
    print(f"📥 {logger_msg}")
    
    return {
        "task_id": task_id,
        "status": "queued",
        "message": logger_msg,
        "queue_position": position,
        "broadcast": request.speak_msg
    }

@app.get("/health")
def health_check():
    return {"status": "ok", "service": "emergency_call_service"}

if __name__ == "__main__":
    print("🚀 启动紧急呼叫服务...")
    print("📡 监听地址: http://0.0.0.0:9000")
    # 启动服务
    uvicorn.run(app, host="0.0.0.0", port=9000)
