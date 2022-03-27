FROM ros:noetic-robot

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_WS_DIR="/closed_loop_ws"
ENV ROS_DISTRO="${ROS_DISTRO}"

WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y install \
    git \
    python3-catkin-tools \
    python3-osrf-pycommon

RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-plugins \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller

RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

ENV DEBIAN_FRONTEND=dialog

COPY "/.vscode" "${ROS_WS_DIR}/.vscode"
RUN sed -i -E "s/(ROS_DISTRO=\\$\{1:-\").*(\"\})/\1${ROS_DISTRO}\2/" ${ROS_WS_DIR}/.vscode/ros_source.bash \
    && echo "alias rsource='source ${ROS_WS_DIR}/.vscode/ros_source.bash'" >> ~/.bash_aliases \
    && echo "alias rbuild='(cd ${ROS_WS_DIR} && catkin build)'" >> ~/.bash_aliases \
    && echo "alias rclean='(cd ${ROS_WS_DIR} && catkin clean -y)'" >> ~/.bash_aliases \
    && echo "source ${ROS_WS_DIR}/.vscode/ros_source.bash" >> ~/.bashrc

