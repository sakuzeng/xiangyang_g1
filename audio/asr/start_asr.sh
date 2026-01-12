#!/bin/bash

# === 配置 ===
# ⚠️ 请确保这里的文件名和你保存的 Python 代码文件名一致
APP_NAME="server_asr.py"
LOG_DIR="./logs"
PID_FILE="asr_service.pid"

# ✅ 日志文件名包含时-分-秒，确保唯一
DATE=$(date +%Y-%m-%d_%H-%M-%S)
LOG_FILE="$LOG_DIR/asr_service_$DATE.log"

# 1. 清理 logs 目录下超过 7 天的 .log 文件
find "$LOG_DIR" -name "*.log" -type f -mtime +7 -exec rm -f {} \; 2>/dev/null

# 2. 检查并创建日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    echo "📂 创建日志目录: $LOG_DIR"
fi

# 3. 检查服务是否已在运行
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p "$OLD_PID" > /dev/null 2>&1; then
        echo "⚠️  ASR 服务已在运行中 (PID: $OLD_PID)"
        echo "   如需重启，请先执行 ./stop_asr.sh"
        exit 1
    else
        echo "🧹 清理残留 PID 文件..."
        rm -f "$PID_FILE"
    fi
fi

# 4. 启动服务
echo "🚀 正在启动 ASR 服务: $APP_NAME"
echo "📝 日志文件: $LOG_FILE"

nohup python -u "$APP_NAME" > "$LOG_FILE" 2>&1 &

# 5. 保存 PID
NEW_PID=$!
echo "$NEW_PID" > "$PID_FILE"

echo "✅ ASR 服务已启动! PID: $NEW_PID"
echo "👉 查看实时日志: tail -f $LOG_FILE"