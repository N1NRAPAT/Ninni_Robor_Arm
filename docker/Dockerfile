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
    ros-humble-cv-bridge \
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

# ── OpenCV CUDA build dependencies ──────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # V4L2 camera support
    libv4l-dev \
    v4l-utils \
    # Image format libs
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libwebp-dev \
    # Video codec libs
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    # GUI (needed for imshow via WSLg)
    libgtk-3-dev \
    # Linear algebra (speeds up CPU fallback ops)
    libeigen3-dev \
    libatlas-base-dev \
    # Python bindings
    python3-dev \
    python3-numpy \
 && rm -rf /var/lib/apt/lists/*

# ── Build OpenCV 4.10.0 from source with CUDA 12.8 + sm_89 (RTX 5080) ──────
ENV OPENCV_VERSION=4.10.0

RUN cd /tmp && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git

RUN mkdir -p /tmp/opencv/build && cd /tmp/opencv/build && \
    cmake .. \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      # CUDA flags
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN="8.9" \
      -D CUDA_ARCH_PTX="8.9" \
      -D CMAKE_CUDA_ARCHITECTURES=89 \
      -D WITH_CUDNN=OFF \
      -D OPENCV_DNN_CUDA=OFF \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      # Extra modules (cuda codecs, tracking, etc.)
      -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
      # Camera & codec support
      -D WITH_V4L=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_GSTREAMER=OFF \
      # GUI for imshow via WSLg
      -D WITH_GTK=ON \
      -D WITH_QT=OFF \
      # Python bindings
      -D BUILD_opencv_python3=ON \
      -D PYTHON3_EXECUTABLE=$(which python3) \
      # Skip unused modules to keep build time reasonable
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_DOCS=OFF \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    # Clean up source to keep image size down
    && rm -rf /tmp/opencv /tmp/opencv_contrib

RUN rosdep init && rosdep update || true

WORKDIR /ros2_ws
COPY ninni_robot_arm /ros2_ws/src/ninni_robot_arm

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=0

COPY start_desktop.sh /start_desktop.sh
RUN chmod +x /start_desktop.sh

EXPOSE 6080 5900

ENTRYPOINT ["/start_desktop.sh"]