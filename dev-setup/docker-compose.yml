# Use ROS2 Humble with Gazebo
FROM osrf/ros:humble-desktop-full

# Install necessary packages
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    novnc \
    websockify \
    nginx \
    supervisor \
    vim \
    wget \
    python3-pip \
    openssh-server \
    sudo \
    git \
    curl \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install VS Code Server
RUN curl -fsSL https://code-server.dev/install.sh | sh

# Set up SSH
RUN mkdir /var/run/sshd
RUN echo 'root:rosdev' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Create development user
RUN useradd -m -s /bin/bash -G sudo devuser && \
    echo "devuser:rosdev" | chpasswd

# Add user to video group for GPU access
RUN usermod -a -G video devuser

# Set up VNC password
RUN mkdir ~/.vnc && x11vnc -storepasswd rosdev ~/.vnc/passwd

# Copy configuration files
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY nginx.conf /etc/nginx/nginx.conf

# Create startup script
COPY start.sh /
RUN chmod +x /start.sh

# Create workspace directory
RUN mkdir -p /home/devuser/ros2_ws
RUN chown -R devuser:devuser /home/devuser/ros2_ws

WORKDIR /home/devuser/ros2_ws

# Set environment variables for Gazebo
ENV DISPLAY=:0
ENV GAZEBO_IP=127.0.0.1
ENV GAZEBO_MASTER_IP=127.0.0.1
ENV GAZEBO_MASTER_URI=http://127.0.0.1:11345
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Start supervisord
CMD ["/usr/bin/supervisord"]