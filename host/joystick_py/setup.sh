#!/usr/bin/env bash
# joystick_py（master 分支 Python / micro-ROS 版）开机自启管理
# 用法: ./setup.sh {install|remove|help}
set -euo pipefail

SERVICE_NAME="joystick_teleop.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
LEGACY_SERVICE="joystick_bringup.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENTRY="${SCRIPT_DIR}/joystick_manual.sh"
TELEOP="${SCRIPT_DIR}/joystick_teleop.py"
SERIAL_DEV="/dev/ttyS4"

usage() {
    cat <<EOF
用法: $(basename "$0") {install|remove|help}

  install  生成 ${SERVICE_PATH}，设为开机自启并立即启动
  remove   停止服务、取消开机自启并删除服务文件
  help     显示本帮助

服务名: ${SERVICE_NAME}
入口:   ${ENTRY} teleop_script:=${TELEOP}

注意:
  * joystick_py 与 joystick_cpp 共用服务名 ${SERVICE_NAME}，
    后一次 install 会覆盖前一次；两者不能同时运行（都要占用 ${SERIAL_DEV}）。
  * install 会停止并禁用遗留的 ${LEGACY_SERVICE}（旧 ROS2 launch 服务），避免重复启动 agent。
EOF
}

require_sudo() {
    if ! sudo -n true 2>/dev/null; then
        echo "需要 sudo 权限写 ${SERVICE_PATH}，请按提示输入密码。"
        sudo true
    fi
}

do_install() {
    if [ ! -x "${ENTRY}" ]; then
        echo "错误: 手动入口不存在或不可执行: ${ENTRY}" >&2
        exit 1
    fi
    if [ ! -f "${TELEOP}" ]; then
        echo "错误: 手柄脚本不存在: ${TELEOP}" >&2
        exit 1
    fi
    if [ ! -e "${SERIAL_DEV}" ]; then
        echo "警告: ${SERIAL_DEV} 不存在，服务启动后可能无法连接 ESP32" >&2
    fi
    require_sudo

    # 旧服务、以及可能被另一版本占用的同名服务，都要先停掉再换单元文件
    if systemctl is-active --quiet "${LEGACY_SERVICE}" 2>/dev/null; then
        echo "停止遗留服务 ${LEGACY_SERVICE} ..."
        sudo systemctl stop "${LEGACY_SERVICE}"
    fi
    if systemctl is-enabled --quiet "${LEGACY_SERVICE}" 2>/dev/null; then
        echo "禁用遗留服务 ${LEGACY_SERVICE} ..."
        sudo systemctl disable "${LEGACY_SERVICE}"
    fi
    if systemctl is-active --quiet "${SERVICE_NAME}" 2>/dev/null; then
        echo "停止当前 ${SERVICE_NAME} ..."
        sudo systemctl stop "${SERVICE_NAME}"
    fi

    local unit
    unit="$(mktemp)"
    cat >"${unit}" <<EOF
[Unit]
Description=Joystick teleop (Python / micro-ROS)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=orangepi
Group=orangepi
WorkingDirectory=${SCRIPT_DIR}
Environment=JOYSTICK_DISPLAY=0
Environment=SDL_VIDEODRIVER=dummy
Environment=SDL_AUDIODRIVER=dummy
Environment=SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1
ExecStart=${ENTRY} teleop_script:=${TELEOP}
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
    sudo install -m 644 "${unit}" "${SERVICE_PATH}"
    rm -f "${unit}"

    sudo systemctl daemon-reload
    sudo systemctl enable --now "${SERVICE_NAME}"
    echo "已安装并启动 ${SERVICE_NAME}"
    systemctl --no-pager --lines=8 status "${SERVICE_NAME}" || true
}

do_remove() {
    require_sudo
    if systemctl is-active --quiet "${SERVICE_NAME}" 2>/dev/null; then
        echo "停止 ${SERVICE_NAME} ..."
        sudo systemctl stop "${SERVICE_NAME}"
    fi
    if systemctl is-enabled --quiet "${SERVICE_NAME}" 2>/dev/null; then
        echo "取消开机自启 ${SERVICE_NAME} ..."
        sudo systemctl disable "${SERVICE_NAME}"
    fi
    if [ -f "${SERVICE_PATH}" ]; then
        sudo rm -f "${SERVICE_PATH}"
        echo "已删除 ${SERVICE_PATH}"
    else
        echo "${SERVICE_PATH} 不存在，跳过删除"
    fi
    sudo systemctl daemon-reload
    sudo systemctl reset-failed "${SERVICE_NAME}" >/dev/null 2>&1 || true
    echo "已移除 ${SERVICE_NAME}"
}

case "${1:-}" in
    install) do_install ;;
    remove)  do_remove ;;
    help|"") usage ;;
    *)
        echo "未知参数: $1" >&2
        usage >&2
        exit 2
        ;;
esac
