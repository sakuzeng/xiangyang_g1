import os
import time
from funasr import AutoModel
from funasr.utils.postprocess_utils import rich_transcription_postprocess

# 1. 定义模型 ID
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

try:
    model = AutoModel(
        model=model_id,
        trust_remote_code=True,
        device="cpu",
        disable_update=True
    )
except Exception as e:
    print(f"\n❌ 模型加载失败: {e}")
    if model_id == local_model_dir:
        print(f"💡 提示: 请检查本地模型文件是否完整: {local_model_dir}")
    else:
        print(f"💡 提示: 请尝试手动下载模型到: {local_model_dir}")
    raise

print("✅ 模型加载成功！")

# 2. 推理测试
input_file = "./data/example2.wav" 

if os.path.exists(input_file):
    print(f"🎤 正在识别: {input_file}")
    
    start_time = time.time()
    
    res = model.generate(
        input=input_file,
        cache={},
        language="zh",  # auto, zh, en, ja, ko, yue
        use_itn=True,     # 逆文本标准化 (例如: "一百" -> "100")
        batch_size_s=60,
        merge_vad=True,
        merge_length_s=15,
    )
    
    end_time = time.time()
    
    # 3. 提取结果
    if res:
        # rich_transcription_postprocess 会自动去除 <|zh|><|happy|> 等情感标签
        text = rich_transcription_postprocess(res[0]["text"])
        print(f"📝 识别结果: {text}")
        print(f"⚡ 推理耗时: {(end_time - start_time)*1000:.2f} ms")
    else:
        print("未识别到内容")

else:
    print(f"⚠️ 未找到 {input_file}")