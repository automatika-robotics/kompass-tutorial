# KOMPASS Tutorial for ROSCon Strasbourg

## Getting Started

1. The easiest way to get started is to get the docker container with all simulation and tutorial resources already installed:

```shell
docker pull ghcr.io/automatika-robotics/kompass-tutorials:humble
```

2. Run your contaier with DISPLAY access (to launch the simulator GUI)


```shell
xhost +
docker run --name=kompass-demos -it --group-add video --volume=/tmp/.X11-unix:/tmp/.X11-unix  --env="DISPLAY=$DISPLAY" ghcr.io/automatika-robotics/kompass-tutorials:humble
```

4. To attach to the container run:

```shell
docker exec -it kompass-demos bash
```

5. To start the test simulation, attach to the container and run:

`source ros_ws/install/setup.bash && ros2 launch kompass_sim webots_turtlebot3.launch.py`

Press 'Y/y' to install Webots on the first run.

6. To start a recipe, attach to the container and run:

`source ros_ws/install/setup.bash && cd /kompass-tutorial && python3 <recipe_file>`


## To inspect the system in ROS2:

- To see the available ROS2 nodes after running a recipe, attache to the container and run:

`source ros_ws/install/setup.bash && ros2 node list`

- To see the available ROS2 topics after running a recipe, attache to the container and run:

`source ros_ws/install/setup.bash && ros2 topic list`

- To see the a topic's data after running a recipe, attache to the container and run:

`source ros_ws/install/setup.bash && ros2 topic echo <topic_name>`
