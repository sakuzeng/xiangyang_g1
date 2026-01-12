#!/bin/bash

# 切换到脚本所在目录
cd "$(dirname "$0")" || exit

PID_FILE="emergency_service.pid"
# ✅ 修正：必须与启动脚本一致
APP_NAME="server_emergency_call.py"

echo "----------------------------------------------------"
echo "🛑 正在停止紧急呼叫服务..."

# 1. 检查 PID 文件
if [ ! -f "$PID_FILE" ]; then
    echo "⚠️  未找到 PID 文件 ($PID_FILE)，尝试通过进程名查找..."
    
    # 兜底：直接查找进程名
    # 移除交互式询问，直接查找并清理
    if pkill -f "python -u $APP_NAME"; then
        echo "✅ 已通过进程名强制停止残留的服务进程。"
    else
        echo "❌ 未找到运行中的服务。"
    fi
    exit 0
fi

# 2. 读取 PID
PID=$(cat "$PID_FILE")

# 3. 检查进程是否存在
if kill -0 "$PID" 2>/dev/null; then

    # === 安全检查：防止 PID 重用 ===
    # 确保这个 PID 跑的真的是我们的 Python 脚本
    if ps -p "$PID" -o args= | grep -q "$APP_NAME"; then
        
        echo "🛑 发送停止信号 (PID: $PID)..."
        # 使用 standard kill (SIGTERM)，让程序有机会处理 finally 块
        kill "$PID" 
        
        # 循环检查 (最多 5 秒)
        for i in {1..5}; do
            if ! kill -0 "$PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        
        # 强制终止
        if kill -0 "$PID" 2>/dev/null; then
            echo "⚠️  进程未响应，强制终止 (kill -9)..."
            kill -9 "$PID"
        fi
        
        rm -f "$PID_FILE"
        echo "✅ 服务已停止。"
        
    else
        echo "⚠️  严重警告：PID $PID 存在，但进程名不匹配 $APP_NAME！"
        echo "⚠️  可能是 PID 被系统其他进程重用。"
        echo "🛑 跳过停止操作，并清理过期的 PID 文件。"
        rm -f "$PID_FILE"
    fi

else
    echo "⚠️  进程已不存在，清理残留 PID 文件。"
    rm -f "$PID_FILE"
fi
echo "----------------------------------------------------"