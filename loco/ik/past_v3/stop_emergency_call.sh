#!/bin/bash

# 切换到脚本所在目录
cd "$(dirname "$0")"

PID_FILE="emergency_service.pid"
APP_NAME="emergency_call_service.py"

# 1. 检查 PID 文件
if [ ! -f "$PID_FILE" ]; then
    echo "⚠️  找不到 PID 文件"
    
    # 尝试查找残留进程
    echo "🔍 正在搜索残留进程..."
    PIDS=$(ps aux | grep "$APP_NAME" | grep -v grep | awk '{print $2}')
    
    if [ -z "$PIDS" ]; then
        echo "✅ 没有发现运行中的服务"
        exit 0
    else
        echo "🔍 发现进程: $PIDS"
        read -p "是否强制终止? (y/n): " confirm
        if [ "$confirm" = "y" ]; then
            echo $PIDS | xargs kill -9
            echo "✅ 已清理"
        fi
        exit 0
    fi
fi

# 2. 读取 PID
PID=$(cat "$PID_FILE")

# 3. 停止进程
if ps -p $PID > /dev/null; then
    echo "🛑 正在停止服务 (PID: $PID)..."
    kill -2 $PID # 发送 SIGINT (相当于 Ctrl+C)
    
    # 等待退出
    for i in {1..5}; do
        if ! ps -p $PID > /dev/null; then
            break
        fi
        sleep 1
    done
    
    # 强制终止
    if ps -p $PID > /dev/null; then
        echo "⚠️  进程未响应，强制终止..."
        kill -9 $PID
    fi
    
    echo "✅ 服务已停止"
else
    echo "⚠️  进程已不存在"
fi

# 清理 PID 文件
rm -f "$PID_FILE"