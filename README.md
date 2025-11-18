# KOMPASS Tutorial for ROSCon Strasbourg

## Getting Started

1. The easiest way to get started is to get the docker container with all simulation and tutorial resources already installed:

```shell
docker pull <IMAGE_NAME>
```

2. Run your contaier with DISPLAY access (to launch the simulator GUI)


```shell
xhost +
docker run --name=kompass-demos -it --group-add video --volume=/tmp/.X11-unix:/tmp/.X11-unix  --env="DISPLAY=$DISPLAY" kompass-tutorials:humble
```

4. To attach to the container run:

```shell
docker exec -it kompass-demos bash
```

5. To start the test simulation, attach to the container and run:

`source ros_ws/install/setup.bash && ros2 launch kompass_sim webots_turtlebot3.launch.py`

Press 'Y/y' to install Webots on the first run.
