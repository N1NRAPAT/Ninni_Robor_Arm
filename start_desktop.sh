#!/bin/bash
set -e
Xvfb :1 -screen 0 800x600x24 &
sleep 1
fluxbox &
x11vnc -display :1 -nopw -forever -shared -rfbport 5900 &
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &
sleep 1
echo "noVNC ready: open http://localhost:6080/vnc.html in your browser"
exec bash
