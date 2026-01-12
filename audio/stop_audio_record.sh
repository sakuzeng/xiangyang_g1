#!/bin/bash

# === 配置 ===
PID_FILE="server_get_audio.pid"
APP_NAME="server_get_audio.py"

# 1. 检查 PID 文件是否存在
if [ ! -f "$PID_FILE" ]; then
    echo "⚠️  找不到 PID 文件，音频服务可能未运行"
    
    # 尝试查找并清理残留进程
    echo "🔍 正在搜索残留进程..."
    PIDS=$(ps aux | grep "$APP_NAME" | grep -v grep | awk '{print $2}')
    
    if [ -z "$PIDS" ]; then
        echo "✅ 没有发现运行中的音频服务进程"
        exit 0
    else
        echo "🔍 发现以下进程:"
        ps aux | grep "$APP_NAME" | grep -v grep
        read -p "是否强制终止这些进程? (y/n): " confirm
        if [ "$confirm" = "y" ]; then
            echo $PIDS | xargs kill -9
            echo "✅ 已强制终止残留进程"
        fi
        exit 0
    fi
fi

# 2. 读取 PID
PID=$(cat "$PID_FILE")

# 3. 检查进程是否存在
if ! ps -p $PID > /dev/null; then
    echo "⚠️  进程 PID:$PID 不存在（可能已经停止）"
    rm "$PID_FILE"
    exit 0
fi

# 4. 优雅停止（SIGINT，相当于 Ctrl+C）
echo "🛑 正在停止音频服务 (PID: $PID) ..."
kill -2 $PID

# 5. 等待进程退出（最多10秒）
for i in {1..10}; do
    if ! ps -p $PID > /dev/null; then
        echo "✅ 音频服务已停止"
        rm "$PID_FILE"
        exit 0
    fi
    echo "⏳ 等待进程退出... ($i/10)"
    sleep 1
done

# 6. 如果还未退出，强制终止
echo "⚠️  进程未响应优雅退出，执行强制终止..."
kill -9 $PID

# 7. 再次检查
sleep 1
if ! ps -p $PID > /dev/null; then
    echo "✅ 进程已强制终止"
    rm "$PID_FILE"
else
    echo "❌ 无法终止进程 $PID，请手动处理"
    exit 1
fi