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
else
    echo ">> Locale en_US.UTF-8 already configured, skipping."
fi
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# --- Universe repository ---
if ! apt-cache policy | grep -q "universe"; then
    echo ">> Enabling universe repository..."
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe
else
    echo ">> Universe repository already enabled, skipping."
fi

# --- ROS 2 apt source ---
if ! dpkg -l ros2-apt-source &>/dev/null; then
    echo ">> Adding ROS 2 apt source..."
    sudo apt-get install -y curl
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    echo ">> Detected ROS apt source version: ${ROS_APT_SOURCE_VERSION}"
    CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
    echo ">> Detected Ubuntu codename: ${CODENAME}"
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    rm -f /tmp/ros2-apt-source.deb
    sudo apt-get update
else
    echo ">> ROS 2 apt source already installed, skipping."
fi

# --- ROS 2 and system packages ---
echo ">> Installing ROS 2 and system packages (already installed packages will be skipped by apt)..."
sudo apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-rosbridge-suite \
    swig \
    build-essential \
    portaudio19-dev \
    alsa-utils \
    espeak-ng \
    libespeak1

# --- Python packages (apt) ---
echo ">> Installing Python packages via apt (already installed packages will be skipped by apt)..."
sudo apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pyaudio \
    python3-requests \
    python3-numpy \
    python3-websockets

# --- Python packages (pip) ---
echo ">> Installing Python pip packages (pocketsphinx, pyttsx3, sounddevice, openai)..."
python3 -m pip install pocketsphinx pyttsx3 sounddevice openai --break-system-packages --ignore-installed

# ============================================================
# Beyond this point: RPi-specific setup (not in Dockerfile)
# ============================================================

# --- Firewall (ufw) ---
if ! command -v ufw &>/dev/null; then
    echo ">> Installing ufw..."
    sudo apt-get install -y ufw
else
    echo ">> ufw already found, skipping."
fi

echo ">> Configuring firewall rules..."
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow 22/tcp comment 'SSH'
sudo ufw allow 9090/tcp comment 'rosbridge WebSocket'
sudo ufw --force enable
echo ">> Firewall configured."

# --- Tailscale ---
if ! command -v tailscale &>/dev/null; then
    echo ">> Tailscale not found, installing..."
    curl -fsSL https://tailscale.com/install.sh | sudo bash
    echo ">> Tailscale installed."
else
    echo ">> Tailscale already installed, skipping installation."
fi

if tailscale status &>/dev/null; then
    echo ">> Tailscale is connected."
else
    echo ">> Tailscale is NOT connected. Run: sudo tailscale up"
fi

echo ">> Setup complete."
