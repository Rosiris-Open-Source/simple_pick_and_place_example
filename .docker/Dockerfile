# Docker - Pull base image
FROM osrf/ros:kilted-desktop-full

# Environment - Setup ARGs and ENV, after FROM reset all args.
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME
ARG USER_PASSWORD
ARG USER_UID
ARG USER_GID
ENV UID=$USER_UID
ENV GID=$USER_GID
ENV USER=$USER_NAME
ENV SHELL=/bin/bash
ENV TZ=Europe/Berlin
ENV TERM=xterm-256color
ENV PATH=$PATH:/home/$USER/.local/bin

# Environment - Setup internationalization and styles.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Packages - Update and Install
RUN apt-get update && apt-get install -y \
    bash-completion \
    curl \
    git \
    lsb-release \
    libgl1 \
    mesa-utils \
    sudo \
    trash-cli \
    tree \
    vim \
    wget \
    build-essential \
    cmake \
    gdb \
    net-tools \
    python3 \
    python3-pip \
    python3-venv \
    ros-dev-tools \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-rqt* \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo (OSRF) repo and key
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable \
      $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# Install gz-ionic (includes gz-sim, gz-gui etc.)
RUN apt-get update && apt-get install -y gz-jetty \
    && rm -rf /var/lib/apt/lists/*

# Make python and pip globally available if missing
RUN [ ! -e /usr/bin/python ] && ln -s /usr/bin/python3 /usr/bin/python || true &&  \
	[ ! -e /usr/bin/pip ] && ln -s /usr/bin/pip3 /usr/bin/pip || true
# Initialize rosdep
RUN rosdep init || true

# Remove existing ubuntu user if it exists and conflicts
RUN if id "ubuntu" &>/dev/null; then \
        if [ "$(id -u ubuntu)" = "${UID}" ] || [ "$(id -g ubuntu)" = "${GID}" ]; then \
            userdel -r ubuntu 2>/dev/null || true; \
            groupdel ubuntu 2>/dev/null || true; \
        fi; \
    fi

# Remove existing group with target GID if it exists
RUN if getent group "${GID}" &>/dev/null; then \
        groupdel "$(getent group "${GID}" | cut -d: -f1)" 2>/dev/null || true; \
    fi

# Create user from Variables.
RUN echo "Creating user ${USER} with UID ${UID} and GID ${GID}..."
RUN groupadd --gid "${GID}" "${USER}" && \
    useradd --uid "${UID}" --gid "${GID}" -m --shell $(which bash) "${USER}" -G sudo && \
    echo "${USER}:${USER_PASSWORD}" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp && \
    chmod 0440 /etc/sudoers.d/sudogrp && \
    chown ${UID}:${GID} -R /home/${USER}

USER $USER
WORKDIR /home/$USER

# Create workspace directory with correct ownership
RUN mkdir -p /home/$USER/workspace
# Source ROS environment automatically
RUN echo "source /opt/ros/kilted/setup.bash" >> /home/$USER/.bashrc
RUN rosdep update || true

# Set up entrypoint
ENTRYPOINT ["/bin/bash", "-c", "exec bash"]