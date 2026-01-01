#!/bin/bash
#
# RMCV 看门狗脚本 v2.0
# 综合 FYT2024 (systemd + 心跳) 和 sp_vision_25 (screen) 的优点
#
# 功能:
#   1. screen 会话管理 (支持调试和日志)
#   2. 心跳文件监控 (检测线程卡死)
#   3. 进程优先级设置 (实时调度 + nice)
#   4. 自动重启和会话目录管理
#
# 用法:
#   ./watchdog.sh              # 普通模式
#   ./watchdog.sh --match      # 比赛模式 (强制内录)
#

# ========== 配置 ==========
TIMEOUT=10                       # 心跳超时 (秒)
MAX_RETRY=100                    # 最大重启次数
SCREEN_NAME="rmcv"               # screen 会话名

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RMCV_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$RMCV_DIR/build"
LOG_DIR="$RMCV_DIR/log"
EXECUTABLE="$BUILD_DIR/RMCV2026"

# 命令行参数
ARGS="$@"

# 进程优先级配置
ENABLE_REALTIME=true             # 启用实时调度
NICE_LEVEL=-15                   # nice 优先级 (-20最高, 19最低)
RT_PRIORITY=50                   # 实时优先级 (1-99, 越高越优先)
# CPU_AFFINITY="0-3"             # CPU 亲和性 (取消注释启用)
ENABLE_COREDUMP=true             # 启用 coredump (崩溃时生成 core 文件)

# 运行时变量
RETRY_COUNT=0
SESSION_DIR=""    # 当前会话目录 (由 create_session_dir 设置)

# ========== 工具函数 ==========

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

get_timestamp() {
    date '+%Y-%m-%d_%H-%M-%S'
}

# 创建新的会话目录
create_session_dir() {
    local timestamp=$(get_timestamp)
    local suffix=""

    # 检查是否有 --match 参数
    if echo "$ARGS" | grep -q "\-\-match\|\-m"; then
        suffix="_match"
    fi

    SESSION_DIR="$LOG_DIR/${timestamp}${suffix}"
    mkdir -p "$SESSION_DIR"

    # 更新 latest 链接
    rm -f "$LOG_DIR/latest"
    ln -sf "$SESSION_DIR" "$LOG_DIR/latest"

    log_msg "Session: $SESSION_DIR"
}

update_session_link() {
    if [ -n "$SESSION_DIR" ]; then
        rm -f "$LOG_DIR/latest"
        ln -sf "$SESSION_DIR" "$LOG_DIR/latest"
    fi
}

# ========== 进程优先级 ==========

set_priority() {
    local pid=$1
    if [ -z "$pid" ]; then return; fi

    # 设置 nice 值 (需要 root 或 CAP_SYS_NICE)
    if renice $NICE_LEVEL -p $pid 2>/dev/null; then
        log_msg "  Nice set to $NICE_LEVEL"
    fi

    # 设置实时调度 (需要 root 或 CAP_SYS_NICE)
    if $ENABLE_REALTIME; then
        if chrt -f -p $RT_PRIORITY $pid 2>/dev/null; then
            log_msg "  RT priority set to SCHED_FIFO:$RT_PRIORITY"
        else
            # 降级尝试 SCHED_RR
            if chrt -r -p $RT_PRIORITY $pid 2>/dev/null; then
                log_msg "  RT priority set to SCHED_RR:$RT_PRIORITY"
            fi
        fi
    fi

    # CPU 亲和性 (可选)
    if [ -n "${CPU_AFFINITY:-}" ]; then
        if taskset -cp $CPU_AFFINITY $pid 2>/dev/null; then
            log_msg "  CPU affinity set to $CPU_AFFINITY"
        fi
    fi
}

# ========== Screen 管理 ==========

kill_screen() {
    if screen -list 2>/dev/null | grep -q "$SCREEN_NAME"; then
        log_msg "Killing existing screen session..."
        screen -S "$SCREEN_NAME" -X quit 2>/dev/null
        sleep 1
    fi
}

start_screen() {
    log_msg "Starting RMCV in screen session '$SCREEN_NAME'..."
    cd "$BUILD_DIR"

    # 创建会话目录 (watchdog 先创建，再传给 RMCV)
    create_session_dir

    # 启用 coredump (崩溃时生成 core 文件到会话目录)
    if $ENABLE_COREDUMP; then
        ulimit -c unlimited
        # 设置 core 文件路径 (需要 root 权限修改 /proc/sys/kernel/core_pattern)
        # echo "$SESSION_DIR/core.%e.%p" | sudo tee /proc/sys/kernel/core_pattern
        log_msg "Coredump enabled (ulimit -c unlimited)"
    fi

    # screen 日志直接写到会话目录
    SCREEN_LOG="$SESSION_DIR/screen.log"

    # 创建 screen 会话，传入 --log-dir 参数
    # RMCV 的 run.log 和 watchdog 的 screen.log 都在同一个目录
    screen -dmS "$SCREEN_NAME" -L -Logfile "$SCREEN_LOG" \
        bash -c "cd \"$BUILD_DIR\" && $EXECUTABLE $ARGS --log-dir \"$SESSION_DIR\" 2>&1; echo '[RMCV exited with code '\$?']'; exec bash"

    sleep 3  # 等待 RMCV 启动

    # 获取 RMCV PID 并设置优先级
    RMCV_PID=$(pgrep -f "RMCV2026" | head -1)
    if [ -n "$RMCV_PID" ]; then
        log_msg "RMCV PID: $RMCV_PID"
        set_priority $RMCV_PID
    fi
}

# ========== 进程/心跳检查 ==========

check_process() {
    pgrep -f RMCV2026 > /dev/null
}

check_screen() {
    screen -list 2>/dev/null | grep -q "$SCREEN_NAME"
}

check_heartbeat() {
    # 使用当前会话目录的心跳文件
    if [ -z "$SESSION_DIR" ]; then
        log_msg "  Session dir not set, waiting..."
        return 0
    fi

    local heartbeat_file="$SESSION_DIR/heartbeat"

    if [ ! -f "$heartbeat_file" ]; then
        log_msg "  Heartbeat file not found, waiting..."
        return 0  # 还没启动完成
    fi

    # 检查文件修改时间
    local file_age=$(($(date +%s) - $(stat -c %Y "$heartbeat_file" 2>/dev/null || echo 0)))

    if [ "$file_age" -gt "$TIMEOUT" ]; then
        log_msg "  Heartbeat TIMEOUT: ${file_age}s > ${TIMEOUT}s"
        return 1
    fi

    log_msg "  Heartbeat OK (age: ${file_age}s)"
    return 0
}

# ========== 启动/重启 ==========

bringup() {
    kill_screen
    pkill -9 -f RMCV2026 2>/dev/null || true
    sleep 1
    start_screen
}

restart() {
    log_msg "Restarting RMCV..."
    RETRY_COUNT=$((RETRY_COUNT + 1))

    if [ "$RETRY_COUNT" -gt "$MAX_RETRY" ]; then
        log_msg "[FATAL] Max retry ($MAX_RETRY) reached!"
        exit 1
    fi

    log_msg "Retry $RETRY_COUNT / $MAX_RETRY"
    bringup
    sleep $((TIMEOUT * 2))  # 重启后等待
}

cleanup() {
    log_msg "Received exit signal, cleaning up..."
    kill_screen
    pkill -f RMCV2026 2>/dev/null || true
    exit 0
}

# ========== 主逻辑 ==========

# 捕获信号
trap cleanup SIGINT SIGTERM

# 检查可执行文件
if [ ! -x "$EXECUTABLE" ]; then
    log_msg "Error: Executable not found: $EXECUTABLE"
    exit 1
fi

# 创建日志目录
mkdir -p "$LOG_DIR"

# 看门狗日志
WATCHDOG_LOG="$LOG_DIR/watchdog_$(date '+%Y%m%d_%H%M%S').log"
exec > >(tee -a "$WATCHDOG_LOG") 2>&1

echo ""
echo "============================================"
echo "  RMCV Watch Dog v2.0"
echo "============================================"
log_msg "RMCV Dir:    $RMCV_DIR"
log_msg "Executable:  $EXECUTABLE"
log_msg "Args:        $ARGS"
log_msg "Timeout:     ${TIMEOUT}s"
log_msg "Max Retry:   $MAX_RETRY"
log_msg "Screen:      $SCREEN_NAME"
log_msg "Realtime:    $ENABLE_REALTIME (nice=$NICE_LEVEL, rt=$RT_PRIORITY)"
echo ""

# 首次启动
bringup
sleep $((TIMEOUT * 2))

# 监控循环
while true; do
    log_msg "--- Health Check ---"

    # 1. 检查 screen 会话
    if ! check_screen; then
        log_msg "  Screen session lost!"
        restart
        continue
    fi

    # 2. 检查进程
    if ! check_process; then
        log_msg "  RMCV process not running!"
        restart
        continue
    fi

    # 3. 检查心跳
    if ! check_heartbeat; then
        log_msg "  Heartbeat check failed!"
        restart
        continue
    fi

    # 一切正常
    RETRY_COUNT=0
    sleep $TIMEOUT
done
