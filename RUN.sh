#!/usr/bin/env bash
#==============================================================================
# RUN.sh — ADAS Fusion 一站式构建 & 启动脚本
#==============================================================================
# 用法:
#   ./RUN.sh              → 一键编译 + 启动 (基础模式)
#   ./RUN.sh build        → 仅编译
#   ./RUN.sh start        → 仅启动 (基础模式)
#   ./RUN.sh advanced     → 编译 + 高级参数启动 (交互式菜单)
#   ./RUN.sh camera       → 编译 (含 depthai 相机驱动, 需 Jetson + libdepthai)
#   ./RUN.sh clean        → 清理编译产物
#   ./RUN.sh deps         → 安装系统依赖
#==============================================================================

set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WS_DIR"

# ---- 颜色 ----
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*"; }
step()  { echo -e "\n${BLUE}==>${NC} $*"; }

# ---- ROS 2 环境检测 ----
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        # 尝试自动 source
        if [ -f /opt/ros/humble/setup.bash ]; then
            source /opt/ros/humble/setup.bash
        elif [ -f /opt/ros/jazzy/setup.bash ]; then
            source /opt/ros/jazzy/setup.bash
        else
            err "ROS 2 未找到. 请先 source /opt/ros/<distro>/setup.bash"
            err "示例: source /opt/ros/humble/setup.bash"
            return 1
        fi
    fi
    info "ROS 2 ${ROS_DISTRO} detected"
    return 0
}

# ---- 系统依赖安装 ----
install_deps() {
    step "Installing system dependencies"
    sudo apt update

    # ROS 2 基础依赖
    sudo apt install -y \
        python3-pip python3-numpy python3-serial \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-joy \
        2>/dev/null || warn "Some apt packages may have failed, continuing..."

    pip3 install --user pyserial numpy 2>/dev/null || warn "pip install may have failed"

    info "Dependencies installed"
}

# ---- 编译 ----
build_all() {
    check_ros2 || return 1

    step "Step 1/3: Build message packages (depthai_ros_msgs, mmw_radar_msgs, adas_fusion_msgs)"

    # 构建消息包 (无额外硬件依赖)
    colcon build --symlink-install \
        --packages-select depthai_ros_msgs mmw_radar_msgs mmw_radar_driver adas_fusion_msgs \
        --event-handlers console_direct+ 2>&1 || {
        warn "Some message packages failed. Trying with --continue-on-error..."
        colcon build --symlink-install --continue-on-error \
            --packages-select depthai_ros_msgs mmw_radar_msgs mmw_radar_driver adas_fusion_msgs
    }

    source install/setup.bash 2>/dev/null || true

    step "Step 2/3: Build fusion & decision package"
    colcon build --symlink-install \
        --packages-select adas_fusion \
        --event-handlers console_direct+ || {
        err "adas_fusion build failed."
        err "Check that vision_msgs is installed: sudo apt install ros-${ROS_DISTRO}-vision-msgs"
        return 1
    }

    source install/setup.bash 2>/dev/null || true

    step "Step 3/3: Build LiDAR driver"
    # rplidar_ros: 无额外依赖, 可在 VM 上构建
    if colcon build --symlink-install \
        --packages-select rplidar_ros \
        --event-handlers console_direct+ 2>/dev/null; then
        info "rplidar_ros built successfully"
    else
        warn "rplidar_ros build skipped (check sllidar_ros2 SDK)"
    fi

    source install/setup.bash 2>/dev/null || true
    info "Build complete"
    echo ""
    info "NOTE: depthai_ros_driver NOT built — requires Jetson + libdepthai + OAK-D"
    info "      To build with camera support: ./RUN.sh camera"
}

# ---- 相机驱动构建 (仅 Jetson 硬件) ----
build_camera() {
    check_ros2 || return 1

    step "Building depthai camera driver (requires libdepthai + OAK-D hardware)"

    if ! dpkg -l libdepthai-dev 2>/dev/null | grep -q '^ii'; then
        err "libdepthai-dev not installed."
        err "Install: sudo apt install libdepthai-dev"
        err "Or follow: https://docs.luxonis.com/software/depthai/installation/"
        return 1
    fi

    colcon build --symlink-install \
        --packages-select depthai_ros_driver \
        --event-handlers console_direct+ || {
        err "depthai_ros_driver build failed"
        return 1
    }

    source install/setup.bash 2>/dev/null || true
    info "Camera driver built — use enable_camera:=true when launching"
}

# ---- 基础启动 ----
basic_launch() {
    check_ros2 || return 1

    if [ ! -f install/setup.bash ]; then
        err "Workspace not built. Run './RUN.sh build' first."
        return 1
    fi

    source install/setup.bash

    step "Launching ADAS Fusion (basic mode)"
    info "Sensors: lidar + radar (default); camera requires OAK-D hardware"
    info "Control: joystick (emergency override enabled)"
    info "TTC thresholds: WARN=5s SLOW=3s EMERG=1s"
    info "To enable camera: ros2 launch adas_fusion system.launch.py enable_camera:=true"
    echo ""

    ros2 launch adas_fusion system.launch.py \
        enable_lidar:=true \
        enable_radar:=true \
        enable_joystick:=true
}

# ---- 高级启动 ----
advanced_launch() {
    check_ros2 || return 1

    if [ ! -f install/setup.bash ]; then
        err "Workspace not built. Run './RUN.sh build' first."
        return 1
    fi

    source install/setup.bash

    echo ""
    echo "============================================"
    echo "  ADAS Fusion — Advanced Launch"
    echo "============================================"
    echo ""
    echo "Select launch mode:"
    echo "  1) Full system (sensors + fusion + decision + serial)"
    echo "  2) Fusion & decision only (sensors already running)"
    echo "  3) Sensors only"
    echo "  4) Custom parameters"
    echo ""
    read -p "Choice [1-4]: " MODE

    case $MODE in
        1)
            read -p "Enable camera? [Y/n]: " CAM
            read -p "Enable LiDAR? [Y/n]: " LID
            read -p "Enable radar? [Y/n]: " RAD
            read -p "Enable joystick? [Y/n]: " JOY

            CAM_VAL="true"; [ "$CAM" = "n" ] || [ "$CAM" = "N" ] && CAM_VAL="false"
            LID_VAL="true"; [ "$LID" = "n" ] || [ "$LID" = "N" ] && LID_VAL="false"
            RAD_VAL="true"; [ "$RAD" = "n" ] || [ "$RAD" = "N" ] && RAD_VAL="false"
            JOY_VAL="true"; [ "$JOY" = "n" ] || [ "$JOY" = "N" ] && JOY_VAL="false"

            ros2 launch adas_fusion system.launch.py \
                enable_camera:=$CAM_VAL \
                enable_lidar:=$LID_VAL \
                enable_radar:=$RAD_VAL \
                enable_joystick:=$JOY_VAL
            ;;
        2)
            ros2 launch adas_fusion fusion_decision.launch.py
            ;;
        3)
            read -p "LiDAR port [/dev/ttyUSB0]: " LPORT; LPORT=${LPORT:-/dev/ttyUSB0}
            read -p "Radar port [/dev/ttyUSB1]: " RPORT; RPORT=${RPORT:-/dev/ttyUSB1}
            ros2 launch adas_fusion sensors.launch.py \
                lidar_port:=$LPORT \
                radar_port:=$RPORT
            ;;
        4)
            echo ""
            echo "Enter custom parameters (press Enter to use defaults):"
            read -p "Max linear velocity [0.3]: " MAXV; MAXV=${MAXV:-0.3}
            read -p "Max angular velocity [0.5]: " MAXW; MAXW=${MAXW:-0.5}
            read -p "TTC warning threshold [5.0]: " TWARN; TWARN=${TWARN:-5.0}
            read -p "TTC slowdown threshold [3.0]: " TSLOW; TSLOW=${TSLOW:-3.0}
            read -p "TTC emergency threshold [1.0]: " TEMERG; TEMERG=${TEMERG:-1.0}
            read -p "Emergency cooldown seconds [3.0]: " COOL; COOL=${COOL:-3.0}
            read -p "Serial port [/dev/ttyTHS2]: " SPORT; SPORT=${SPORT:-/dev/ttyTHS2}
            read -p "Enable joystick? [Y/n]: " JOYE
            JOY_VAL="true"; [ "$JOYE" = "n" ] || [ "$JOYE" = "N" ] && JOY_VAL="false"

            ros2 launch adas_fusion system.launch.py \
                max_linear_vel:=$MAXV \
                max_angular_vel:=$MAXW \
                ttc_warning:=$TWARN \
                ttc_slowdown:=$TSLOW \
                ttc_emergency:=$TEMERG \
                cooldown_seconds:=$COOL \
                serial_port:=$SPORT \
                enable_joystick:=$JOY_VAL
            ;;
        *)
            err "Invalid choice"
            return 1
            ;;
    esac
}

# ---- 清理 ----
clean_ws() {
    step "Cleaning build artifacts"
    rm -rf build/ install/ log/
    info "Cleaned"
}

# ---- 菜单 ----
show_menu() {
    echo ""
    echo "============================================"
    echo "  ADAS Fusion — Build & Run Script"
    echo "============================================"
    echo ""
    echo "  ./RUN.sh build      仅编译 (LiDAR + Radar + Fusion)"
    echo "  ./RUN.sh camera     编译 (含深度相机驱动, 需 Jetson)"
    echo "  ./RUN.sh start      基础启动 (默认参数)"
    echo "  ./RUN.sh advanced   高级启动 (交互式参数)"
    echo "  ./RUN.sh            编译 + 基础启动 (一键)"
    echo "  ./RUN.sh deps       安装系统依赖"
    echo "  ./RUN.sh clean      清理编译产物"
    echo ""
}

# ---- 主入口 ----
main() {
    cmd="${1:-all}"

    case "$cmd" in
        build)
            build_all
            ;;
        camera)
            build_all && build_camera
            ;;
        start)
            basic_launch
            ;;
        advanced)
            build_all && advanced_launch
            ;;
        deps)
            install_deps
            ;;
        clean)
            clean_ws
            ;;
        all|"")
            show_menu
            read -p "Press Enter to build + launch, or Ctrl+C to cancel..."
            install_deps
            build_all
            basic_launch
            ;;
        *)
            show_menu
            err "Unknown command: $cmd"
            ;;
    esac
}

main "$@"
