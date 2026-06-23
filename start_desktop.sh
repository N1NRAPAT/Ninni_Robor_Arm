#!/bin/bash
set -e

# Start virtual display
Xvfb :1 -screen 0 1920x1080x24&
sleep 2

# Start window manager
fluxbox &
sleep 1

# Start VNC server
x11vnc -display :1 -nopw -forever -shared -rfbport 5900 &

# Start noVNC websocket bridge
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &
sleep 1

echo "=============================="
echo "noVNC ready at:"
echo "http://localhost:6080/vnc.html"
echo "=============================="

# Source ROS2
source /opt/ros/humble/setup.bash

# Keep container alive
exec bash