#!/bin/bash
#
# RMCV systemd 服务安装脚本
#
# 用法: sudo ./install_service.sh
#

set -e

# 检查 root 权限
if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 请使用 sudo 运行"
    echo "用法: sudo ./install_service.sh"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RMCV_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$RMCV_DIR/build"
SERVICE_FILE="$SCRIPT_DIR/rmcv.service"

# 获取实际用户
REAL_USER="${SUDO_USER:-$USER}"
REAL_GROUP="$(id -gn $REAL_USER)"
REAL_HOME="$(eval echo ~$REAL_USER)"

echo ""
echo "============================================"
echo "  RMCV Service Installer"
echo "============================================"
echo "User:     $REAL_USER"
echo "Group:    $REAL_GROUP"
echo "RMCV Dir: $RMCV_DIR"
echo ""

# 检查文件
if [ ! -f "$SERVICE_FILE" ]; then
    echo "错误: 找不到 $SERVICE_FILE"
    exit 1
fi

if [ ! -f "$BUILD_DIR/RMCV2026" ]; then
    echo "警告: 可执行文件不存在 $BUILD_DIR/RMCV2026"
    echo "请先编译: cd $RMCV_DIR && mkdir -p build && cd build && cmake .. && make -j"
    read -p "是否继续安装? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 设置脚本权限
chmod +x "$SCRIPT_DIR/watchdog.sh"
chmod +x "$SCRIPT_DIR/cleanup.sh" 2>/dev/null || true

# 创建 service 文件 (替换路径和用户)
echo "创建 service 文件..."
TMP_SERVICE="/tmp/rmcv.service"
sed -e "s|User=rmcv|User=$REAL_USER|g" \
    -e "s|Group=rmcv|Group=$REAL_GROUP|g" \
    -e "s|/home/rmcv/RMCV|$RMCV_DIR|g" \
    "$SERVICE_FILE" > "$TMP_SERVICE"

# 复制到 systemd 目录
cp "$TMP_SERVICE" /etc/systemd/system/rmcv.service
chmod 644 /etc/systemd/system/rmcv.service

# 设置实时优先级权限 (可选)
echo "配置实时优先级权限..."
LIMITS_FILE="/etc/security/limits.d/rmcv.conf"
cat > "$LIMITS_FILE" << EOF
# RMCV 实时优先级权限
$REAL_USER    -    rtprio    99
$REAL_USER    -    nice      -20
$REAL_USER    -    memlock   unlimited
EOF
chmod 644 "$LIMITS_FILE"

# 重新加载 systemd
echo "重新加载 systemd..."
systemctl daemon-reload

# 启用服务
echo "启用开机自启..."
systemctl enable rmcv.service

echo ""
echo "============================================"
echo "  安装完成!"
echo "============================================"
echo ""
echo "使用方法:"
echo "  sudo systemctl start rmcv    # 启动 (比赛模式)"
echo "  sudo systemctl stop rmcv     # 停止"
echo "  sudo systemctl restart rmcv  # 重启"
echo "  sudo systemctl status rmcv   # 查看状态"
echo "  journalctl -u rmcv -f        # 查看日志"
echo ""
echo "调试:"
echo "  screen -r rmcv               # 连接到 RMCV 终端"
echo "  screen -d rmcv               # 断开 (不关闭)"
echo ""
echo "手动运行:"
echo "  $SCRIPT_DIR/watchdog.sh --match   # 比赛模式"
echo "  $SCRIPT_DIR/watchdog.sh           # 普通模式"
echo ""

# 询问是否立即启动
read -p "是否立即启动服务? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl start rmcv
    echo "服务已启动!"
    systemctl status rmcv --no-pager
fi
