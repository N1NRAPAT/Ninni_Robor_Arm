# ROS2 Humble, arm64-native image — works directly on Apple Silicon (M1/M2/M3/M4)
# without emulation, which matters a lot for build speed.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# --- System + ROS deps ---
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

RUN rosdep update || true

WORKDIR /ros2_ws

# Source ROS automatically in every new shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

ENTRYPOINT ["/bin/bash"]
