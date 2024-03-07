FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash" , "-c" ]
# Install packages
RUN apt-get update && apt-get install -y \
    terminator \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3* \
    python3-colcon-common-extensions \
    ros-humble-turtlebot4-desktop \
    git \
    && apt-get clean

RUN apt-get clean all
# Install Git
RUN apt-get update && apt-get install -y git

RUN source /opt/ros/humble/setup.bash && \
    mkdir -p /turtlebot3_ws/src && \
    cd /turtlebot3_ws/src && \
    git clone https://github.com/vinay06vinay/Turtlebot3-Obstacle-Avoidance-ROS2.git

# Set environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle_pi" >>/root/.bashrc

# Build the Ros2 workspace workspace and ensure it's sourced
RUN source /opt/ros/humble/setup.bash \
    && cd turtlebot3_ws \
    && colcon build 
RUN echo "source /turtlebot3_ws/install/setup.bash" >> /root/.bashrc

# Set the working folder at startup
WORKDIR /turtlebot3_ws