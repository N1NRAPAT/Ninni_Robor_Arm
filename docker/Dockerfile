FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    build-essential \
    cmake \
    git \
    ros-humble-demo-nodes-cpp \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    fluxbox \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep update || true

WORKDIR /ros2_ws
COPY ninni_robot_arm /ros2_ws/src/ninni_robot_arm

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DISPLAY=:1

COPY start_desktop.sh /start_desktop.sh
RUN chmod +x /start_desktop.sh

EXPOSE 6080 5900

ENTRYPOINT ["/start_desktop.sh"]
