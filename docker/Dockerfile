FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-demo-nodes-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

# CUDA Python support
RUN pip3 install --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cu128

# noVNC + virtual display
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    fluxbox \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update || true

WORKDIR /ros2_ws
COPY ninni_robot_arm /ros2_ws/src/ninni_robot_arm

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV DISPLAY=:1
# Use real GPU rendering now (no software fallback)
ENV LIBGL_ALWAYS_SOFTWARE=0

COPY start_desktop.sh /start_desktop.sh
RUN chmod +x /start_desktop.sh

EXPOSE 6080 5900

ENTRYPOINT ["/start_desktop.sh"]