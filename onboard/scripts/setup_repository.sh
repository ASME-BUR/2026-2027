#!/bin/bash
# Jazzy port of bur-onboard/scripts/setup_repository.sh
#
# Changes from last year:
#   - humble -> jazzy, Ubuntu 22.04 -> 24.04
#   - gazebo packages DROPPED. Simulation moved to MuJoCo, outside this workspace.
#   - mavros DROPPED. Nothing in the tree ever used it.
#   - pigpio DROPPED. The only driver that needed it (Bar30) now talks to
#     /dev/i2c-1 directly. See ../README.md, "Hardware notes".
#   - python3-vcs2l -> python3-vcstool (vcs2l is not in the Noble archive)
set -e
ROS_DISTRO=${ROS_DISTRO:-jazzy}

grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc \
    || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

source /opt/ros/${ROS_DISTRO}/setup.bash

# NOTE: Ubuntu's Raspberry Pi images ship with noble-updates and noble-backports
# disabled, and the ROS 2 apt packages need them. On a Pi, check this first:
#   grep Suites /etc/apt/sources.list.d/ubuntu.sources
# and make sure the first entry reads: Suites: noble noble-updates noble-backports

apt-get update
apt-get install -y ros-${ROS_DISTRO}-robot-localization \
                   ros-${ROS_DISTRO}-behaviortree-cpp \
                   ros-${ROS_DISTRO}-robot-state-publisher \
                   ros-${ROS_DISTRO}-xacro \
                   ros-${ROS_DISTRO}-joy \
                   ros-${ROS_DISTRO}-teleop-twist-joy \
                   ros-${ROS_DISTRO}-v4l2-camera \
                   ros-${ROS_DISTRO}-tf2-ros \
                   ros-${ROS_DISTRO}-tf2-geometry-msgs \
                   python3-pip \
                   python3-rosdep \
                   python3-vcstool \
                   python3-colcon-common-extensions \
                   libi2c-dev \
                   i2c-tools \
                   libeigen3-dev \
                   build-essential \
                   cmake \
                   git \
                   nano \
                   unzip \
                   wget

# No nav2 packages, on purpose (DEVELOPMENT.md D-7): bur_autonomy replaced the
# two helpers it used. Don't add navigation2 here.

[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
rosdep update
