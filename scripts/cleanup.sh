#!/bin/bash
#
# RMCV 清理脚本
#
# 用法: ./cleanup.sh
#

echo "[$(date '+%H:%M:%S')] Cleaning up RMCV..."

# 停止 systemd 服务 (如果存在)
if systemctl is-active --quiet rmcv 2>/dev/null; then
    echo "Stopping systemd service..."
    sudo systemctl stop rmcv
fi

# 杀掉 screen 会话
if screen -list 2>/dev/null | grep -q rmcv; then
    echo "Killing screen session..."
    screen -S rmcv -X quit 2>/dev/null
fi

# 杀掉看门狗
if pgrep -f "watchdog.sh" > /dev/null; then
    echo "Killing watchdog..."
    pkill -f "watchdog.sh"
fi

# 杀掉 RMCV 进程
if pgrep -f "RMCV2026" > /dev/null; then
    echo "Killing RMCV..."
    pkill -9 -f "RMCV2026"
fi

sleep 1

echo "[$(date '+%H:%M:%S')] Cleanup done."

# 显示状态
echo ""
echo "进程检查:"
pgrep -a -f "RMCV2026|watchdog" || echo "  (无 RMCV 相关进程)"
