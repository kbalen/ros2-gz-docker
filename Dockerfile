# Use an official ROS 2 base image
FROM osrf/ros:humble-desktop

# Install Gazebo packages, xvfb, and x11-xserver-utils (if needed)
RUN apt-get update && \
    apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    xvfb \
    fluxbox \
    x11vnc \
    tomcat9 \
    wget \
    supervisor \
    guacd \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglx-mesa0 \
    libglu1-mesa \
    libegl1 \
    libopengl0 \
    ros-humble-turtlebot3* \
    x11-xserver-utils \
    default-jdk && \
    rm -rf /var/lib/apt/lists/*

# Source ROS2 in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc && \
    echo "export GAZEBO_MASTER_URI=http://127.0.0.1:11345" >> ~/.bashrc

# Configure Tomcat properly
RUN mkdir -p /usr/share/tomcat9/conf && \
    mkdir -p /usr/share/tomcat9/logs && \
    mkdir -p /usr/share/tomcat9/webapps && \
    cp /etc/tomcat9/server.xml /usr/share/tomcat9/conf/ && \
    cp /etc/tomcat9/web.xml /usr/share/tomcat9/conf/ && \
    cp -r /var/lib/tomcat9/conf/* /usr/share/tomcat9/conf/ && \
    chown -R tomcat:tomcat /usr/share/tomcat9

# Set environment variables
ENV DISPLAY=:99
ENV TURTLEBOT3_MODEL=burger
ENV GUACAMOLE_HOME=/etc/guacamole
ENV CATALINA_HOME=/usr/share/tomcat9
ENV CATALINA_BASE=/var/lib/tomcat9

# Download and deploy the Guacamole web application
RUN wget -O /var/lib/tomcat9/webapps/guacamole.war "https://downloads.apache.org/guacamole/1.5.3/binary/guacamole-1.5.3.war" && \
    chown tomcat:tomcat /var/lib/tomcat9/webapps/guacamole.war && \
    chmod 644 /var/lib/tomcat9/webapps/guacamole.war

# Create the Guacamole configuration directory and copy config files
RUN mkdir -p /etc/guacamole && \
    chown -R tomcat:tomcat /etc/guacamole

COPY guacamole.properties /etc/guacamole/guacamole.properties
COPY user-mapping.xml /etc/guacamole/user-mapping.xml

RUN chown tomcat:tomcat /etc/guacamole/guacamole.properties && \
    chown tomcat:tomcat /etc/guacamole/user-mapping.xml && \
    chmod 644 /etc/guacamole/guacamole.properties && \
    chmod 644 /etc/guacamole/user-mapping.xml

# Copy the supervisord configuration file
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Source Gazebo in .bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$HOME/ros_ws/src" >> ~/.bashrc

# Create ROS workspace
RUN mkdir -p /root/ros_ws/src

# Set GAZEBO_MODEL_PATH environment variable
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/root/ros_ws/src

# Expose the necessary ports
EXPOSE 8080 4822 5900

# Use supervisord to launch and manage all processes
CMD ["/usr/bin/supervisord", "-n"]
