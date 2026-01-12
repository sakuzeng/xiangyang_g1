#!/bin/bash

# === 配置 ===
APP_NAME="integrated_wake_recorder.py"
LOG_DIR="./logs"
PID_FILE="interaction_system.pid"
DEFAULT_INTERFACE="eth0"

# 获取命令行传入的网卡名称，如果没有传入则使用默认值
INTERFACE=${1:-$DEFAULT_INTERFACE}

# 清理 logs 目录下超过 7 天的 .log 文件
if [ -d "$LOG_DIR" ]; then
    find "$LOG_DIR" -name "interaction_*.log" -type f -mtime +7 -exec rm -f {} \;
fi

# 生成带时间戳的日志文件名
DATE=$(date +%Y-%m-%d_%H-%M-%S)
LOG_FILE="$LOG_DIR/interaction_$DATE.log"

# 1. 检查日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    echo "📂 创建日志目录: $LOG_DIR"
fi

# 2. 检查是否已经在运行
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p $OLD_PID > /dev/null; then
        echo "⚠️  语音交互系统已经在运行中 (PID: $OLD_PID)"
        echo "   如需重启,请先执行 ./stop_interaction.sh"
        exit 1
    else
        # PID 文件存在但进程不存在(僵尸文件),清理掉
        rm "$PID_FILE"
    fi
fi

# 3. 检查依赖服务状态
echo "🔍 检查依赖服务..."

# 检查 ASR 服务
ASR_STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://192.168.77.103:28003/ 2>/dev/null)
if [ "$ASR_STATUS" = "200" ]; then
    echo "✅ ASR 服务正常 (192.168.77.103:28003)"
else
    echo "⚠️  ASR 服务连接失败,系统将继续启动但功能可能受限"
fi

# 检查 TTS 服务
TTS_STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://192.168.77.103:28001/ 2>/dev/null)
if [ "$TTS_STATUS" = "200" ]; then
    echo "✅ TTS 服务正常 (192.168.77.103:28001)"
else
    echo "⚠️  TTS 服务连接失败,系统将继续启动但功能可能受限"
fi

# 4. 启动服务
echo "🚀 正在启动语音交互系统 (网卡: $INTERFACE) ..."
echo "📝 日志文件: $LOG_FILE"

# nohup 后台运行,传入网卡参数
nohup python -u $APP_NAME $INTERFACE > "$LOG_FILE" 2>&1 &

# 5. 获取并保存 PID
NEW_PID=$!
echo $NEW_PID > "$PID_FILE"

# 6. 等待 2 秒检查进程是否正常启动
sleep 2
if ps -p $NEW_PID > /dev/null; then
    echo "✅ 语音交互系统已启动! PID: $NEW_PID"
    echo "👉 查看实时日志: tail -f $LOG_FILE"
    echo "👉 停止服务: ./stop_interaction.sh"
    echo ""
    echo "🎯 系统状态:"
    echo "   - 唤醒词: 小安"
    echo "   - 模式: 唤醒检测 → 用户录音 → 识别回复"
    echo "   - ASR: 远程服务 (192.168.77.103:28003)"
    echo "   - TTS: 流式播放 (192.168.77.103:28001)"
else
    echo "❌ 启动失败! 请查看日志: $LOG_FILE"
    rm "$PID_FILE"
    exit 1
fi