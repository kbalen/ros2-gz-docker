# Use the official ROS Humble Desktop image as base.
FROM ros:humble-perception-jammy

# Set noninteractive mode for apt-get.
ENV DEBIAN_FRONTEND=noninteractive

# Add VirtualGL and TurboVNC repositories and install their GPG keys
RUN apt-get update && apt-get install -y wget gpg && \
    wget -q -O- https://packagecloud.io/dcommander/virtualgl/gpgkey | gpg --dearmor > /etc/apt/trusted.gpg.d/VirtualGL.gpg && \
    wget -q -O "/etc/apt/sources.list.d/VirtualGL.list" "https://raw.githubusercontent.com/VirtualGL/repo/main/VirtualGL.list" && \
    wget -q -O- "https://packagecloud.io/dcommander/turbovnc/gpgkey" | gpg --dearmor > "/etc/apt/trusted.gpg.d/TurboVNC.gpg" && \
    wget -q -O "/etc/apt/sources.list.d/TurboVNC.list" "https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list" 
# Install Gazebo, ROS Gazebo packages, X packages, noVNC, websockify, VirtualGL, TurboVNC, and supervisor.
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    mesa-utils \
    openbox \
    xterm \
    novnc \
    websockify \
    virtualgl \
    turbovnc \
    supervisor \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up VNC password and xstartup
RUN mkdir -p /root/.vnc && \
    echo '#!/bin/bash\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\n\nexport XDG_RUNTIME_DIR="/tmp/runtime-root"\nmkdir -p $XDG_RUNTIME_DIR\nchmod 700 $XDG_RUNTIME_DIR\n\n# Start window manager explicitly\nopenbox &\n\n# Keep the script running\nexec sleep infinity' > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup && \
    echo "ros" | /opt/TurboVNC/bin/vncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd

# Create directories for logs and supervisor
RUN mkdir -p /var/log/supervisor && \
    mkdir -p /var/run/supervisor && \
    mkdir -p /root/logs

COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Expose the necessary ports:
#   5901 for VNC and 6080 for noVNC (websockify).
EXPOSE 5901 6080

# Set environment variables for Gazebo
ENV DISPLAY=:1
ENV TURTLEBOT3_MODEL=burger

# Source ROS2 and Gazebo in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 

# Start supervisord (which in turn starts all the configured processes).
CMD ["/usr/bin/supervisord", "-n"]

