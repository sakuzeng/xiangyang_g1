#!/bin/bash

PID_FILE="asr_service.pid"
APP_NAME="server_asr.py"

# 1. 检查 PID 文件是否存在
if [ ! -f "$PID_FILE" ]; then
    echo "⚠️  未找到 PID 文件 ($PID_FILE)，服务可能未运行。"
    # 兜底：尝试通过进程名查找
    echo "🔍 尝试通过进程名查找 ASR 服务..."
    if pkill -f "python -u $APP_NAME"; then
        echo "✅ 已通过进程名强制停止 ASR 服务。"
    else
        echo "❌ 未找到运行中的 ASR 服务。"
    fi
    exit 0
fi

# 2. 读取 PID 并检查进程是否存在
PID=$(cat "$PID_FILE")
if ps -p "$PID" > /dev/null 2>&1; then
    echo "🛑 正在停止 ASR 服务 (PID: $PID)..."
    kill "$PID"

    # 等待 3 秒优雅退出
    sleep 3
    if ps -p "$PID" > /dev/null 2>&1; then
        echo "⚠️  进程未响应，强制终止..."
        kill -9 "$PID"
    fi

    # 清理 PID 文件
    rm -f "$PID_FILE"
    echo "✅ ASR 服务已停止。"
else
    echo "⚠️  进程 $PID 不存在，清理残留 PID 文件。"
    rm -f "$PID_FILE"
fi