#!/usr/bin/env bash
# Idempotent setup script for Raspberry Pi running Ubuntu Server 24.04
# Mirrors the Docker environment for the Echo ROS 2 project
set -euo pipefail

# --- Locale ---
if ! locale -a 2>/dev/null | grep -q "en_US.utf8"; then
    echo ">> Configuring locale..."
    sudo apt-get update
    sudo apt-get install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
fi
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# --- Universe repository ---
if ! apt-cache policy | grep -q "universe"; then
    echo ">> Enabling universe repository..."
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe
fi

# --- ROS 2 apt source ---
if ! dpkg -l ros2-apt-source &>/dev/null; then
    echo ">> Adding ROS 2 apt source..."
    sudo apt-get install -y curl
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    rm -f /tmp/ros2-apt-source.deb
    sudo apt-get update
fi

# --- ROS 2 and system packages ---
echo ">> Installing ROS 2 and system packages..."
sudo apt-get install -y \
    ros-kilted-ros-base \
    swig \
    build-essential \
    portaudio19-dev \
    alsa-utils \
    espeak-ng \
    libespeak1

# --- Python packages (apt) ---
echo ">> Installing Python packages (apt)..."
sudo apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pyaudio \
    python3-requests \
    python3-numpy \
    python3-websockets

# --- Python packages (pip) ---
echo ">> Installing Python pip packages..."
python3 -m pip install pocketsphinx pyttsx3 sounddevice openai --break-system-packages --ignore-installed

echo ">> Setup complete."
