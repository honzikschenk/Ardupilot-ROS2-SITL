# ArduPilot SITL + ROS 2 + Gazebo Harmonic

The Docker setup provides:

- ROS 2 Jazzy (desktop)
- Gazebo Harmonic (gz sim) + ros-gz bridge
- ArduPilot SITL

## Build the image

```sh
./docker/build.sh
```

## Run the container with GUI support

- macOS:

  1) Install XQuartz from <https://www.xquartz.org/> and start it.
  2) In XQuartz Preferences > Security, enable: "Allow connections from network clients".
  3) Quit and restart XQuartz to apply the setting. Optionally run `xhost + 127.0.0.1` in XQuartz terminal to allow local connections.

- Linux:

  1) Ensure X11 is available (is it desktop and not server?).

```sh
./docker/run.sh
```

Notes:

- If you still get authorization errors on macOS:
  - Ensure XQuartz is running and "Allow connections from network clients" is enabled; then restart XQuartz.
  - In an XQuartz Terminal, run: `xhost +localhost` (or `xhost + 127.0.0.1`) to temporarily allow local TCP clients.
  - Verify host X works (outside Docker): `xeyes` should display a window.
  - Re-run `./docker/run.sh`.
- Linux: Ensure X11 is available. The script passes `$DISPLAY` and mounts `/tmp/.X11-unix` automatically. If access is denied, run `xhost +local:docker` on the host.

## Useful commands inside the container

- Start ArduPilot SITL (ArduCopter) with console and map:

```sh
run_sitl_copter.sh
```

- Launch Gazebo GUI (empty window):

```sh
run_gz_gui.sh
```

- Example: Bridge ROS <-> Gazebo:

```sh
# In a separate terminal inside the container
ros2 run ros_gz_bridge parameter_bridge
```
