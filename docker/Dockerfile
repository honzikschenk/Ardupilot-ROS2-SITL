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

# Setup rosdep
RUN rosdep init || true && rosdep update

# Create workspace directory inside container & non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000
# Create group/user only if the UID/GID are not already present in the base image
RUN set -eux; \
    if ! getent group "${USER_GID}" >/dev/null; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    else \
        # Reuse existing group name for provided GID
        EXISTING_GROUP_NAME="$(getent group ${USER_GID} | cut -d: -f1)"; \
        if [ "${EXISTING_GROUP_NAME}" != "${USERNAME}" ]; then \
            echo "GID ${USER_GID} already in use by group ${EXISTING_GROUP_NAME}, will use that name."; \
            USERNAME="${EXISTING_GROUP_NAME}"; \
        fi; \
    fi; \
    if ! id -u "${USER_UID}" >/dev/null 2>&1; then \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}"; \
    else \
        echo "UID ${USER_UID} already exists, reusing existing user."; \
    fi; \
    mkdir -p /workspace/src; \
    chown -R "${USER_UID}:${USER_GID}" /workspace

# Source ROS setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

COPY --chown=${USERNAME}:${USERNAME} . /workspace

RUN bash -lc 'if [ -d /workspace/ros2_ws/src ]; then cd /workspace/ros2_ws; colcon build || true; fi'

# Helpful X11 env defaults
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /home/${USERNAME}/.bashrc \
 && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

USER ${USERNAME}

CMD ["bash"]
