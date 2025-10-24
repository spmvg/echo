FROM ubuntu:24.04

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install locales and utilities
RUN apt-get update && \
    apt-get install -y locales curl && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set locale environment variables
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Allow universe repository
RUN apt install -y software-properties-common && \
    add-apt-repository universe

# Add ROS 2 GPG key and repository
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2
RUN apt-get update && \
    apt-get install -y ros-kilted-ros-base swig build-essential portaudio19-dev

# Install Python dependencies
RUN apt-get install -y python3-colcon-common-extensions python3-pocketsphinx python3-pyaudio python3-coqui-tts

# Source ROS 2 setup script by default
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/kilted/setup.bash" >> /root/.bashrc

CMD ["bash"]
