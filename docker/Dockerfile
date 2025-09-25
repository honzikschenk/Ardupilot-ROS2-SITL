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
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo (gz) Harmonic repository and key
RUN set -eux; \
        wget -qO - https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg; \
        echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/gazebo-stable.list; \
        apt-get update; \
        apt-get install -y \
            gz-harmonic \
            ros-jazzy-ros-gz \
            ros-jazzy-mavros \
            ros-jazzy-mavros-extras \
            ros-jazzy-geographic-msgs \
            ros-jazzy-vision-opencv \
            && rm -rf /var/lib/apt/lists/*

# Initialize GeographicLib datasets required by MAVROS
RUN geographiclib-get-geoids best; geographiclib-get-gravity egm96; geographiclib-get-magnetic emm2015

# Setup rosdep
RUN rosdep init || true && rosdep update

# Create workspace directory inside container
RUN mkdir -p /workspace/src

# Build and install ArduPilot SITL
ENV ARDUPILOT_HOME=/opt/ardupilot
ENV GZ_VERSION=harmonic
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
            python3-numpy \
            rapidjson-dev \
            libopencv-dev \
            libgstreamer1.0-dev \
            libgstreamer-plugins-base1.0-dev \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-libav \
            gstreamer1.0-gl; \
        rm -rf /var/lib/apt/lists/*; \
        pip3 install --break-system-packages --no-cache-dir MAVProxy; \
        cd ${ARDUPILOT_HOME}; \
        ./waf configure --board sitl; \
        ./waf -j"$(nproc)" copter

# Build ardupilot_gazebo plugin
ENV ARDUPILOT_GZ_HOME=/opt/ardupilot_gazebo
RUN set -eux; \
    git clone https://github.com/ArduPilot/ardupilot_gazebo ${ARDUPILOT_GZ_HOME}; \
    mkdir -p ${ARDUPILOT_GZ_HOME}/build; cd ${ARDUPILOT_GZ_HOME}/build; \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo; \
    make -j"$(nproc)"; \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=${ARDUPILOT_GZ_HOME}/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> /root/.bashrc; \
    echo "export GZ_SIM_RESOURCE_PATH=${ARDUPILOT_GZ_HOME}/models:${ARDUPILOT_GZ_HOME}/worlds:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc

# Convenience: expose sim_vehicle.py and add simple runners
RUN ln -sf ${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py /usr/local/bin/sim_vehicle.py \
    && printf "#!/usr/bin/env bash\nset -e\ncd ${ARDUPILOT_HOME}\n./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \"$@\"\n" > /usr/local/bin/run_sitl_copter.sh \
    && printf "#!/usr/bin/env bash\nset -e\n# Launch Gazebo Harmonic (GUI + server) with default iris runway world\nexec gz sim -r iris_runway.sdf \"$@\"\n" > /usr/local/bin/run_gz_gui.sh \
    && printf "#!/usr/bin/env bash\nset -e\n# Convenience combined launch (SITL + Gazebo)\n( run_gz_gui.sh & ); sleep 5; run_sitl_copter.sh\n" > /usr/local/bin/run_ardupilot_gz.sh \
    && chmod +x /usr/local/bin/run_sitl_copter.sh /usr/local/bin/run_gz_gui.sh /usr/local/bin/run_ardupilot_gz.sh

# Source ROS setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

# Colcon workspace placeholder build script (user will mount actual workspace)
COPY --chown=root:root . /workspace

RUN bash -lc 'if [ -d /workspace/ros2_ws/src ]; then cd /workspace/ros2_ws; colcon build || true; fi'

# Helpful X11 env defaults
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace

CMD ["/bin/bash"]
