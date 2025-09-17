FROM ros:jazzy-ros-core

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install essential ROS 2 tools and dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
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
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo (gz) Harmonic repository and key
RUN set -eux; \
        wget -qO - https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg; \
        echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/gazebo-stable.list; \
        apt-get update; \
        apt-get install -y \
            gz-harmonic \
            ros-jazzy-ros-gz \
            && rm -rf /var/lib/apt/lists/*

# Setup rosdep
RUN rosdep init || true && rosdep update

# Create workspace directory inside container
RUN mkdir -p /workspace/src

# Build and install ArduPilot SITL
ENV ARDUPILOT_HOME=/opt/ardupilot
RUN set -eux; \
        # Clone ArduPilot
        git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ${ARDUPILOT_HOME}; \
        # Install build prerequisites directly (the upstream script refuses to run as root in Docker)
        apt-get update; \
        apt-get install -y --no-install-recommends \
            ccache \
            gawk \
            cmake \
            ninja-build \
            pkg-config \
            libtool \
            autoconf \
            automake \
            libffi-dev \
            libreadline-dev \
            libbz2-dev \
            libssl-dev \
            libsqlite3-dev \
            zlib1g-dev \
            libxml2-dev \
            libxslt1-dev \
            libsdl2-dev \
            libsdl2-image-dev \
            python3-dev \
            python3-setuptools \
            python3-wheel \
            python3-future \
            python3-pexpect \
            python3-lxml \
            python3-empy \
            python3-numpy; \
        rm -rf /var/lib/apt/lists/*; \
        # Python tooling (avoid upgrading apt-managed setuptools/wheel to prevent uninstall errors)
        pip3 install --break-system-packages --no-cache-dir MAVProxy; \
        # Build SITL (ArduCopter as example)
        cd ${ARDUPILOT_HOME}; \
        ./waf configure --board sitl; \
        ./waf -j"$(nproc)" copter

# Convenience: expose sim_vehicle.py and add simple runners
RUN ln -sf ${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py /usr/local/bin/sim_vehicle.py \
    && printf "#!/usr/bin/env bash\nset -e\ncd ${ARDUPILOT_HOME}\n./Tools/autotest/sim_vehicle.py -v ArduCopter --map --console \"$@\"\n" > /usr/local/bin/run_sitl_copter.sh \
    && printf "#!/usr/bin/env bash\nset -e\n# Launch Gazebo Harmonic (GUI + server)\nexec gz sim \"$@\"\n" > /usr/local/bin/run_gz_gui.sh \
    && chmod +x /usr/local/bin/run_sitl_copter.sh /usr/local/bin/run_gz_gui.sh

# Source ROS setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Helpful X11 env defaults
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace

CMD ["/bin/bash"]
