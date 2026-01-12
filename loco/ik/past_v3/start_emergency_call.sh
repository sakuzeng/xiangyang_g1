#!/bin/bash

# 切换到脚本所在目录，确保相对路径正确
cd "$(dirname "$0")"

# === 配置 ===
APP_NAME="server_emergency_call.py"
LOG_DIR="./logs"
PID_FILE="emergency_service.pid"

# 清理 logs 目录下超过 7 天的日志
if [ -d "$LOG_DIR" ]; then
    find "$LOG_DIR" -name "emergency_service_*.log" -type f -mtime +7 -exec rm -f {} \;
fi

# 生成带时间戳的日志文件名
DATE=$(date +%Y-%m-%d_%H-%M-%S)
LOG_FILE="$LOG_DIR/emergency_service_$DATE.log"

# 1. 检查日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    echo "📂 创建日志目录: $LOG_DIR"
fi

# 2. 检查是否已经在运行
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p $OLD_PID > /dev/null; then
        echo "⚠️  紧急呼叫服务已经在运行中 (PID: $OLD_PID)"
        echo "   如需重启,请先执行 ./stop_emergency_service.sh"
        exit 1
    else
        # 僵尸文件清理
        rm "$PID_FILE"
    fi
fi

# 3. 启动服务
echo "🚀 正在启动紧急呼叫服务..."
echo "📝 日志文件: $LOG_FILE"

# nohup 后台运行
# -u: 禁用缓冲，保证日志实时可见
nohup python -u $APP_NAME > "$LOG_FILE" 2>&1 &

# 4. 获取并保存 PID
NEW_PID=$!
echo $NEW_PID > "$PID_FILE"

# 5. 等待 2 秒检查进程是否正常启动
sleep 2
if ps -p $NEW_PID > /dev/null; then
    echo "✅ 服务已启动! PID: $NEW_PID"
    echo "📡 监听端口: 9000"
    echo "👉 查看实时日志: tail -f $LOG_FILE"
    echo "👉 停止服务: ./stop_emergency_service.sh"
else
    echo "❌ 启动失败! 请查看日志: $LOG_FILE"
    rm "$PID_FILE"
    exit 1
fi