FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install strictly what is needed + GUI tools, skip bloat, clean immediately
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-topic-tools \
    ros-jazzy-nav2-bringup \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-image-geometry \
    libpcap-dev \
    python3-colcon-common-extensions \
    xterm \
    micro \
    tree \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS 2 in interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]
