#!/bin/bash
#
# RMCV 看门狗脚本 v2.1
# 综合 FYT2024 (systemd + 心跳) 和 sp_vision_25 (screen) 的优点
#
# 功能:
#   1. screen 会话管理 (支持调试和日志)
#   2. 心跳文件监控 (检测线程卡死)
#   3. 进程优先级设置 (实时调度 + nice)
#   4. 自动重启和会话目录管理
#   5. 资源监控 (CPU/内存/虚拟内存)
#
# 用法:
#   ./watchdog.sh              # 普通模式
#   ./watchdog.sh --match      # 比赛模式 (强制内录)
#

# ========== 配置 ==========
TIMEOUT=15                       # 心跳超时 (秒), 需 > C++ watchdog_node (5s超时 + 10s等待)
MAX_RETRY=100                    # 最大重启次数
SCREEN_NAME="rmcv"               # screen 会话名

# 资源监控配置
RESOURCE_LOG_INTERVAL=5          # 资源记录间隔 (秒)

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
RESOURCE_LOG=""   # 资源日志文件路径
LAST_RESOURCE_LOG_TIME=0  # 上次资源记录时间

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
    RMCV_PID=$(pgrep -x RMCV2026 | head -1)
    if [ -n "$RMCV_PID" ]; then
        log_msg "RMCV PID: $RMCV_PID"
        set_priority $RMCV_PID
    fi
}

# ========== 进程/心跳检查 ==========

check_process() {
    pgrep -x RMCV2026 > /dev/null
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

# ========== 资源监控 ==========

# 初始化资源日志文件
init_resource_log() {
    if [ -z "$SESSION_DIR" ]; then
        return
    fi

    RESOURCE_LOG="$SESSION_DIR/resources.csv"

    # 写入 CSV 头
    echo "timestamp,rmcv_cpu%,rmcv_rss_mb,rmcv_vsz_mb,sys_cpu%,sys_mem_used_mb,sys_mem_total_mb,sys_swap_used_mb,sys_swap_total_mb,cpu_temp_c" > "$RESOURCE_LOG"
    log_msg "Resource log: $RESOURCE_LOG"
}

# 记录资源使用情况
log_resources() {
    local current_time=$(date +%s)

    # 检查是否到达记录间隔
    if [ $((current_time - LAST_RESOURCE_LOG_TIME)) -lt "$RESOURCE_LOG_INTERVAL" ]; then
        return
    fi
    LAST_RESOURCE_LOG_TIME=$current_time

    if [ -z "$RESOURCE_LOG" ] || [ ! -f "$RESOURCE_LOG" ]; then
        init_resource_log
        if [ -z "$RESOURCE_LOG" ]; then
            return
        fi
    fi

    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    local rmcv_cpu=0
    local rmcv_rss=0
    local rmcv_vsz=0
    local sys_cpu=0
    local sys_mem_used=0
    local sys_mem_total=0
    local sys_swap_used=0
    local sys_swap_total=0
    local cpu_temp=0

    # 获取 RMCV 进程资源
    local pid=$(pgrep -x RMCV2026 | head -1)
    if [ -n "$pid" ]; then
        # ps 输出: %CPU RSS VSZ (RSS/VSZ 单位 KB)
        local ps_output=$(ps -p "$pid" -o %cpu=,rss=,vsz= 2>/dev/null | tr -s ' ')
        if [ -n "$ps_output" ]; then
            rmcv_cpu=$(echo "$ps_output" | awk '{print $1}')
            rmcv_rss=$(echo "$ps_output" | awk '{printf "%.1f", $2/1024}')  # KB -> MB
            rmcv_vsz=$(echo "$ps_output" | awk '{printf "%.1f", $3/1024}')  # KB -> MB
        fi
    fi

    # 获取系统资源 (区分 Linux 和 macOS)
    if [ "$(uname)" = "Linux" ]; then
        # Linux: 使用 /proc/stat 和 free
        # 系统 CPU (简化: 取 1 秒采样的 idle 差值)
        local cpu_line=$(head -1 /proc/stat)
        local cpu_idle=$(echo "$cpu_line" | awk '{print $5}')
        local cpu_total=$(echo "$cpu_line" | awk '{print $2+$3+$4+$5+$6+$7+$8}')
        # 简化处理: 用 top 的瞬时值
        sys_cpu=$(top -bn1 | grep "Cpu(s)" | awk '{print 100-$8}' 2>/dev/null || echo "0")

        # 内存信息
        local mem_info=$(free -m 2>/dev/null)
        if [ -n "$mem_info" ]; then
            sys_mem_total=$(echo "$mem_info" | awk '/^Mem:/ {print $2}')
            sys_mem_used=$(echo "$mem_info" | awk '/^Mem:/ {print $3}')
            sys_swap_total=$(echo "$mem_info" | awk '/^Swap:/ {print $2}')
            sys_swap_used=$(echo "$mem_info" | awk '/^Swap:/ {print $3}')
        fi
    else
        # macOS: 使用 vm_stat 和 sysctl
        local page_size=$(pagesize 2>/dev/null || echo 4096)

        # 内存信息
        local vm_stat_output=$(vm_stat 2>/dev/null)
        if [ -n "$vm_stat_output" ]; then
            local pages_free=$(echo "$vm_stat_output" | awk '/Pages free:/ {gsub(/\./,"",$3); print $3}')
            local pages_active=$(echo "$vm_stat_output" | awk '/Pages active:/ {gsub(/\./,"",$3); print $3}')
            local pages_inactive=$(echo "$vm_stat_output" | awk '/Pages inactive:/ {gsub(/\./,"",$3); print $3}')
            local pages_wired=$(echo "$vm_stat_output" | awk '/Pages wired down:/ {gsub(/\./,"",$4); print $4}')
            local pages_compressed=$(echo "$vm_stat_output" | awk '/Pages occupied by compressor:/ {gsub(/\./,"",$5); print $5}')

            sys_mem_total=$(sysctl -n hw.memsize 2>/dev/null | awk '{printf "%.0f", $1/1024/1024}')
            local used_pages=$((pages_active + pages_wired + pages_compressed))
            sys_mem_used=$((used_pages * page_size / 1024 / 1024))

            # Swap
            local swap_info=$(sysctl -n vm.swapusage 2>/dev/null)
            if [ -n "$swap_info" ]; then
                sys_swap_total=$(echo "$swap_info" | awk -F'[ M=]+' '{print int($2)}')
                sys_swap_used=$(echo "$swap_info" | awk -F'[ M=]+' '{print int($4)}')
            fi
        fi

        # CPU (使用 ps 获取系统总 CPU)
        sys_cpu=$(ps -A -o %cpu | awk '{s+=$1} END {printf "%.1f", s}' 2>/dev/null || echo "0")
    fi

    # 获取 CPU 温度
    if [ "$(uname)" = "Linux" ]; then
        # Linux: 读取 thermal_zone (毫摄氏度)
        if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
            cpu_temp=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")
        fi
    else
        # macOS: 需要 osx-cpu-temp 工具，没有则跳过
        if command -v osx-cpu-temp &>/dev/null; then
            cpu_temp=$(osx-cpu-temp 2>/dev/null | awk '{print $1}' || echo "0")
        fi
    fi

    # 写入 CSV
    echo "$timestamp,$rmcv_cpu,$rmcv_rss,$rmcv_vsz,$sys_cpu,$sys_mem_used,$sys_mem_total,$sys_swap_used,$sys_swap_total,$cpu_temp" >> "$RESOURCE_LOG"
}

# ========== 启动/重启 ==========

bringup() {
    kill_screen
    # 先发 SIGTERM 让进程优雅退出，等待 2 秒后再强制杀死
    pkill -TERM -x RMCV2026 2>/dev/null || true
    sleep 2
    pkill -9 -x RMCV2026 2>/dev/null || true
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
echo "  RMCV Watch Dog v2.1"
echo "============================================"
log_msg "RMCV Dir:    $RMCV_DIR"
log_msg "Executable:  $EXECUTABLE"
log_msg "Args:        $ARGS"
log_msg "Timeout:     ${TIMEOUT}s"
log_msg "Max Retry:   $MAX_RETRY"
log_msg "Screen:      $SCREEN_NAME"
log_msg "Realtime:    $ENABLE_REALTIME (nice=$NICE_LEVEL, rt=$RT_PRIORITY)"
log_msg "Resource:    every ${RESOURCE_LOG_INTERVAL}s"
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

    # 4. 记录资源使用
    log_resources

    # 一切正常
    RETRY_COUNT=0
    sleep $TIMEOUT
done
