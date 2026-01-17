FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# =====================
# 1. FIX LỖI ROS FOXY EOL (Quan trọng)
# =====================
RUN rm -f /etc/apt/sources.list.d/ros2-snapshots.list && \
    apt-get update || echo "Update failed, continuing..." && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# =====================
# 2. DEPENDENCIES
# =====================
# Ở đây ta cài 'ros-foxy-tf-transformations' bằng apt
# Đây chính là thư viện python tf_transformations chuẩn của ROS
RUN apt-get update && apt-get install -y \
    git \
    nano \
    vim \
    python3-pip \
    libeigen3-dev \
    tmux \
    ros-foxy-rviz2 \
    ros-foxy-nav2-bringup \
    ros-foxy-nav2-lifecycle-manager \
    ros-foxy-ackermann-msgs \
    ros-foxy-xacro \
    ros-foxy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Cài đặt thư viện Python
# --- SỬA Ở ĐÂY: Chỉ cài transforms3d, bỏ tf-transformations đi ---
RUN pip3 install transforms3d

# =====================
# 3. F1TENTH GYM
# =====================
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

# =====================
# 4. ROS2 WORKSPACE BUILD
# =====================
WORKDIR /sim_ws

RUN mkdir -p /sim_ws/src/f1tenth_gym_ros
COPY . /sim_ws/src/f1tenth_gym_ros

RUN source /opt/ros/foxy/setup.bash && \
    cd /sim_ws && \
    apt-get update && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

# =====================
# 5. ENVIRONMENT SETUP
# =====================
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "source /sim_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]