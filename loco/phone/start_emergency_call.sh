#!/bin/bash

# 1. 切换到脚本所在目录 (防止路径错误)
cd "$(dirname "$0")" || exit

# === 配置 ===
# ⚠️ 必须与停止脚本中的 APP_NAME 保持一致
APP_NAME="server_emergency_call.py"
LOG_DIR="./logs"
PID_FILE="emergency_service.pid"

# 2. 日志管理
# 清理 logs 目录下超过 7 天的日志
if [ -d "$LOG_DIR" ]; then
    find "$LOG_DIR" -name "emergency_service_*.log" -type f -mtime +7 -exec rm -f {} \; 2>/dev/null
fi

# 生成带时间戳的日志文件名
DATE=$(date +%Y-%m-%d_%H-%M-%S)
LOG_FILE="$LOG_DIR/emergency_service_$DATE.log"

# 创建日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    echo "📂 创建日志目录: $LOG_DIR"
fi

echo "----------------------------------------------------"
echo "🚀 正在启动紧急呼叫服务: $APP_NAME"

# 3. 检查是否已经在运行
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if kill -0 "$OLD_PID" 2>/dev/null; then
        echo "⚠️  服务已经在运行中 (PID: $OLD_PID)"
        echo "   如需重启, 请先执行 ./stop_emergency_service.sh"
        exit 1
    else
        echo "🧹 发现过期的 PID 文件，正在清理..."
        rm "$PID_FILE"
    fi
fi

# 4. 启动服务
echo "📝 日志文件: $(pwd)/$LOG_FILE"

# nohup 后台运行
# -u: 禁用 Python 输出缓冲
nohup python -u "$APP_NAME" > "$LOG_FILE" 2>&1 &
NEW_PID=$!

# 5. 启动后检查 (Wait & Check)
sleep 2

if kill -0 "$NEW_PID" 2>/dev/null; then
    # --- 成功 ---
    echo "$NEW_PID" > "$PID_FILE"
    echo "✅ 服务已启动! PID: $NEW_PID"
    echo "📡 请确保端口 (如 9000) 未被占用"
    echo "👉 查看实时日志: tail -f $(pwd)/$LOG_FILE"
else
    # --- 失败 ---
    echo "❌ 启动失败! 进程在 2 秒内退出。"
    echo "👇 错误日志摘要："
    echo "----------------------------------------------------"
    tail -n 10 "$LOG_FILE"
    echo "----------------------------------------------------"
    exit 1
fi
echo "----------------------------------------------------"