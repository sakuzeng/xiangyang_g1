#!/bin/bash

# === 配置 ===
APP_NAME="server_get_audio.py"   # 请修改为你实际的 Python 文件名
LOG_DIR="./logs"
PID_FILE="server_get_audio.pid" # 独立的 PID 文件，避免冲突
DEFAULT_INTERFACE="eth0"     # 默认网卡名称

# 获取命令行传入的网卡名称，如果没有传入则使用默认值
INTERFACE=${1:-$DEFAULT_INTERFACE}

# 清理 logs 目录下超过 7 天的 .log 文件
if [ -d "$LOG_DIR" ]; then
    find "$LOG_DIR" -name "audio_service_*.log" -type f -mtime +7 -exec rm -f {} \;
fi

# 生成带时间戳的日志文件名
DATE=$(date +%Y-%m-%d_%H-%M-%S)
LOG_FILE="$LOG_DIR/audio_service_$DATE.log"

# 1. 检查日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    echo "📂 创建日志目录: $LOG_DIR"
fi

# 2. 检查是否已经在运行
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p $OLD_PID > /dev/null; then
        echo "⚠️  音频服务已经在运行中 (PID: $OLD_PID)"
        echo "   如需重启，请先执行 ./stop_audio.sh"
        exit 1
    else
        # PID 文件存在但进程不存在（僵尸文件），清理掉
        rm "$PID_FILE"
    fi
fi

# 3. 启动服务
echo "🚀 正在启动 $APP_NAME (网卡: $INTERFACE) ..."
echo "📝 日志文件: $LOG_FILE"

# nohup 后台运行，传入网卡参数
nohup python -u $APP_NAME $INTERFACE > "$LOG_FILE" 2>&1 &

# 4. 获取并保存 PID
NEW_PID=$!
echo $NEW_PID > "$PID_FILE"

echo "✅ 音频服务已启动! PID: $NEW_PID"
echo "👉 查看实时日志: tail -f $LOG_FILE"