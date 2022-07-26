FROM ros:galactic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_WS_DIR="/closed_loop_ws"
ENV ROS_DISTRO="${ROS_DISTRO}"

WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y install \
    git \
    python3-osrf-pycommon

RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins

RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins

ENV DEBIAN_FRONTEND=dialog

COPY "/.vscode" "${ROS_WS_DIR}/.vscode"
RUN sed -i -E "s/(ROS_DISTRO=\\$\{1:-\").*(\"\})/\1${ROS_DISTRO}\2/" ${ROS_WS_DIR}/.vscode/ros2_source.bash \
    && echo "alias rsource='source ${ROS_WS_DIR}/.vscode/ros2_source.bash'" >> ~/.bash_aliases \
    && echo "alias rbuild='(cd ${ROS_WS_DIR} && colcon build)'" >> ~/.bash_aliases \
    && echo "source ${ROS_WS_DIR}/.vscode/ros2_source.bash" >> ~/.bashrc

