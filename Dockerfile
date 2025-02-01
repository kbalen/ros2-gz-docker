# Use an official ROS 2 base image
FROM osrf/ros:humble-desktop

# Install Gazebo packages, xvfb, and x11-xserver-utils (if needed)
RUN apt-get update && \
    apt-get install -y ros-humble-gazebo-ros-pkgs xvfb x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 packages
RUN apt-get update && \
    apt-get install -y ros-humble-turtlebot3* && \
    rm -rf /var/lib/apt/lists/*

# Set environment for Xvfb
ENV DISPLAY=:99

# Set TurtleBot3 model
ENV TURTLEBOT3_MODEL=burger

# Source ROS 2 setup script and add Xvfb startup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "# Automatically start Xvfb on display :99 if not already running" >> ~/.bashrc && \
    echo "if ! pgrep Xvfb > /dev/null; then" >> ~/.bashrc && \
    echo "  # Remove stale lock file if present and no process is holding it" >> ~/.bashrc && \
    echo "  if [ -f /tmp/.X99-lock ] && ! fuser /tmp/.X99-lock > /dev/null; then" >> ~/.bashrc && \
    echo "    echo 'Removing stale lock file /tmp/.X99-lock'" >> ~/.bashrc && \
    echo "    rm /tmp/.X99-lock" >> ~/.bashrc && \
    echo "  fi" >> ~/.bashrc && \
    echo "  echo 'Starting Xvfb on display :99...'" >> ~/.bashrc && \
    echo "  Xvfb :99 -screen 0 1400x900x24 &" >> ~/.bashrc && \
    echo "  sleep 1" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "export DISPLAY=:99" >> ~/.bashrc

# Remove the custom CMD so the container simply opens a bash shell
CMD ["/bin/bash"]
