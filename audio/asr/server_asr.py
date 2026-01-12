import os
import sys
import asyncio
import uvicorn
import time
import logging
import tempfile
import shutil
from fastapi import FastAPI, UploadFile, File, HTTPException
from contextlib import asynccontextmanager
from concurrent.futures import ThreadPoolExecutor

# ==========================================
# 1. 环境配置与导入
# ==========================================
# 设置 FunASR 缓存路径 (可选)
os.environ["MODELSCOPE_CACHE"] = "/home/devuser/.cache/modelscope"

from funasr import AutoModel
from funasr.utils.postprocess_utils import rich_transcription_postprocess

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ASR-Service")

# ==========================================
# 2. 全局状态与并发控制
# ==========================================
items = {}  # 存放 ASR 模型实例
gpu_lock = asyncio.Lock()  # 确保 GPU 推理串行化
executor = ThreadPoolExecutor(max_workers=1)  # 单线程池执行阻塞推理

# ==========================================
# 3. 同步 ASR 推理函数（在线程池中运行）
# ==========================================
def _run_asr_sync(audio_path: str):
    """
    在线程池中执行 ASR 推理（阻塞操作）
    """
    model = items["asr_model"]
    start_time = time.time()
    
    # 执行推理
    # SenseVoiceSmall 支持 auto, zh, en, ja, ko, yue
    res = model.generate(
        input=audio_path,
        cache={},
        language="zh", 
        use_itn=True,  # 逆文本标准化 (例如: "一百" -> "100")
        batch_size_s=60,
        merge_vad=True,
        merge_length_s=15,
    )
    
    end_time = time.time()
    cost_ms = (end_time - start_time) * 1000

    # 提取并清洗结果
    final_text = ""
    if res:
        # rich_transcription_postprocess 去除情感标签 <|zh|><|happy|>
        raw_text = res[0]["text"]
        final_text = rich_transcription_postprocess(raw_text)
    
    return final_text, cost_ms

# ==========================================
# 4. FastAPI 生命周期管理
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("=== 🚀 正在启动 ASR 服务 ===")
    
    # 指定本地模型路径
    local_model_dir = os.path.expanduser("~/.cache/modelscope/models/iic/SenseVoiceSmall")
    local_model_dir = os.path.abspath(local_model_dir)

    if os.path.exists(local_model_dir):
        model_id = local_model_dir
        print(f"📂 使用本地模型: {model_id}")
    else:
        print(f"⚠️ 警告: 本地模型路径不存在: {local_model_dir}")
        print("🔄 将尝试使用 'iic/SenseVoiceSmall' 从 ModelScope 在线加载...")
        model_id = "iic/SenseVoiceSmall"
    
    # 加载模型
    try:
        print(f"🔄 正在加载 FunASR 模型: {model_id} (Device: GPU)...")
        model = AutoModel(
            model=model_id,
            trust_remote_code=True,
            device="cpu",
            disable_update=True
        )
        items["asr_model"] = model
        print("✅ ASR 模型加载成功！")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        logger.error(f"模型加载失败: {e}")
        sys.exit(1)

    # 模型预热（可选，随便跑个空推理或者简单载入）
    try:
        print("🔥 正在进行模型预热...")
        # 这里的预热可以用一个极短的空音频或者简单调用，防止第一次卡顿
        # 由于音频构造比较麻烦，这里简单跳过，第一次请求可能会稍慢
        pass 
    except Exception as e:
        print(f"⚠️ 预热警告: {e}")

    yield  # 服务运行中

    # 清理资源
    items.clear()
    executor.shutdown()
    print("=== 👋 ASR 服务已停止 ===")

# ==========================================
# 5. FastAPI 应用定义
# ==========================================
app = FastAPI(title="SenseVoice ASR Service", lifespan=lifespan)

@app.get("/")
def health_check():
    return {"status": "ok", "service": "FunASR-SenseVoice", "device": "GPU"}

@app.post("/asr", summary="上传音频进行语音识别")
async def asr_predict(file: UploadFile = File(...)):
    """
    异步 ASR 接口：接收音频文件（wav/mp3等），返回识别文本。
    内部自动排队使用 GPU，确保显存安全。
    """
    if "asr_model" not in items:
        raise HTTPException(status_code=500, detail="ASR 服务未就绪（模型未加载）")

    # 创建临时文件保存上传的音频
    # FunASR 的 generate 接口通常接受文件路径作为输入最稳定
    file_suffix = os.path.splitext(file.filename)[1] or ".wav"
    
    # 使用 NamedTemporaryFile 创建临时文件，delete=False 确保我们能在关闭后再次打开读取
    with tempfile.NamedTemporaryFile(delete=False, suffix=file_suffix) as tmp_file:
        tmp_path = tmp_file.name
        try:
            # 写入数据
            content = await file.read()
            tmp_file.write(content)
            file_size = len(content) / 1024
            print(f"📥 [请求入队] 接收到音频: {file.filename} | 大小: {file_size:.2f} KB")
        except Exception as e:
            os.remove(tmp_path) # 清理
            logger.error(f"音频保存失败: {e}")
            raise HTTPException(status_code=400, detail="音频文件解析失败")

    # 获取 GPU 锁并执行 ASR
    try:
        async with gpu_lock:
            print("⚡ [获得 GPU 锁] 开始 ASR 推理...")
            loop = asyncio.get_running_loop()
            
            # 在线程池中运行同步推理
            text, cost_ms = await loop.run_in_executor(executor, _run_asr_sync, tmp_path)

            if not text:
                print(f"🔍 [完成] 未识别到内容 (耗时: {cost_ms:.2f}ms)")
            else:
                # 截断日志防止过长
                log_text = text if len(text) < 50 else text[:50] + "..."
                print(f"✅ [完成] 识别成功: {log_text} (耗时: {cost_ms:.2f}ms)")

            return {
                "text": text,
                "cost_ms": cost_ms,
                "filename": file.filename
            }

    except Exception as e:
        print(f"❌ ASR 推理异常: {e}")
        logger.error(f"ASR 推理异常: {e}")
        raise HTTPException(status_code=500, detail="ASR 推理失败")
    
    finally:
        # 无论成功失败，最后都要清理临时文件
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception as cleanup_err:
                logger.warning(f"临时文件清理失败: {cleanup_err}")

# ==========================================
# 6. 启动入口
# ==========================================
if __name__ == "__main__":
    print("📌 启动 ASR 服务 (端口: 8003，GPU 模式)")
    print("💡 注意：workers=1，高并发请求将自动排队使用 GPU")
    uvicorn.run(app, host="0.0.0.0", port=8003, workers=1)