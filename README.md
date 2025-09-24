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

```sh
# Run first terminal
./docker/run.sh


# Run more terminals
sudo docker exec -it ardupilot-ros-sitl-ws-container /bin/bash
```

## Useful commands inside the container

- Start ArduPilot SITL (ArduCopter):

```sh
run_sitl_copter.sh
```

- Launch Gazebo GUI (empty window):

```sh
run_gz_gui.sh
```

- Example: Bridge ROS <-> Gazebo:

```sh
ros2 run ros_gz_bridge parameter_bridge
```
