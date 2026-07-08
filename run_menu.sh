#!/bin/bash

WS_DIR="$HOME/Ninni_Robot_Arm"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="$WS_DIR/install/setup.bash"

source_env() {
    source "$ROS_SETUP"
    if [ -f "$INSTALL_SETUP" ]; then
        source "$INSTALL_SETUP"
    else
        echo "  install/setup.bash not found — run Full Build first."
        return 1
    fi
}

full_build() {
    cd "$WS_DIR" || { echo "Workspace not found at $WS_DIR"; return 1; }
    colcon build
    source_env
}

run_servo_bridge() {
    source_env || return 1
    ros2 run ninni_robot_arm servo_bridge_node.py
}

run_camera_node() {
    source_env || return 1
    # TODO: replace with actual executable name from camera_src/
    ros2 run ninni_robot_arm camera_node.py
}

run_cuda_cv() {
    source_env || return 1
    ros2 run ninni_robot_arm cuda_cv_node.py
}

run_rviz() {
    source_env || return 1
    rviz2
}

show_usbipd_reminder() {
    cat <<'EOF'

Run these in Windows PowerShell (Admin) BEFORE using this script:
  usbipd attach --wsl --busid 1-2   # Waveshare adapter (servos)
  usbipd attach --wsl --busid 1-3   # ST-Link (STM32)

EOF
}

while true; do
    echo ""
    echo "===== Ninni Robot Arm — Launch Menu ====="
    echo "1) Full build (colcon build + source)"
    echo "2) Source workspace only"
    echo "3) Run servo bridge node"
    echo "4) Run camera node"
    echo "5) Run CUDA CV node"
    echo "6) Launch RViz2"
    echo "7) Show usbipd attach reminder (run in Windows PowerShell)"
    echo "8) Exit"
    echo "=========================================="
    read -rp "Choose an option [1-8]: " choice

    case "$choice" in
        1) full_build ;;
        2) source_env ;;
        3) run_servo_bridge ;;
        4) run_camera_node ;;
        5) run_cuda_cv ;;
        6) run_rviz ;;
        7) show_usbipd_reminder ;;
        8) echo "Bye."; exit 0 ;;
        *) echo "Invalid choice." ;;
    esac
done