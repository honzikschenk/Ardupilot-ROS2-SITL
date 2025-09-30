FROM ros:jazzy-ros-core

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install essential ROS 2 tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-cartographer-rviz \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    sudo \
    build-essential \
    curl \
    wget \
    gnupg \
    lsb-release \
    vim \
    tmux \
    less \
    net-tools \
    iputils-ping \
    xauth \
    x11-apps \
    xterm \
    mesa-utils \
    libgl1-mesa-dri \
    libglu1-mesa \
    libxkbcommon-x11-0 \
    python3-tk \
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize GeographicLib datasets required by MAVROS
RUN geographiclib-get-geoids best && geographiclib-get-gravity egm96 && geographiclib-get-magnetic emm2015

RUN rosdep init || true && rosdep update

ARG USERNAME=root
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -eux; \
    if getent group "${USER_GID}" >/dev/null; then \
        EXISTING_GROUP_NAME="$(getent group ${USER_GID} | cut -d: -f1)"; \
        if [ "${EXISTING_GROUP_NAME}" != "${USERNAME}" ]; then \
            echo "GID ${USER_GID} taken by ${EXISTING_GROUP_NAME}; creating group ${USERNAME} with arbitrary GID"; \
            groupadd "${USERNAME}" || true; \
        fi; \
    else \
        groupadd -g "${USER_GID}" "${USERNAME}"; \
    fi; \
    if id -u "${USER_UID}" >/dev/null 2>&1; then \
        echo "UID ${USER_UID} in use; creating ${USERNAME} with next available UID"; \
        useradd -m -g "${USERNAME}" "${USERNAME}" || true; \
    else \
        useradd -m -u "${USER_UID}" -g "${USERNAME}" "${USERNAME}"; \
    fi; \
    mkdir -p /workspace/src; \
    chown -R "${USERNAME}:${USERNAME}" /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

COPY --chown=${USERNAME}:${USERNAME} . /workspace

RUN bash -lc 'if [ -d /workspace/ros2_ws/src ]; then cd /workspace/ros2_ws; colcon build || true; fi'

# Helpful X11 env defaults
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc \
 && chown ${USERNAME}:${USERNAME} /root/.bashrc

USER ${USERNAME}

CMD ["bash"]
