# swarm_mission_planning_srv
A ROS2 service that plans missions for swarms.

## Installation

This is assuming you already have Ubuntu 22.04 and ROS2 Humble

```console
mkdir -p mission_planning_ws/src
cd mission_planning_ws/src
git clone git@github.com:halehaka/swarm_mission_planning_srv.git
git clone git@github.com:halehaka/map_cover.git
git clone git@github.com:halehaka/interfaces_swarm.git
source /opt/ros/humble/setup.bash
colcon build
source  install/setup.bash
```

To have some maps to play with, run
```console
mkdir -p ~/mission_planning_ws/landcover
cd ~/mission_planning_ws/landcover
wget https://landcover.ai.linuxpolska.com/download/landcover.ai.v1.zip
unzip landcover.ai.v1.zip
```

## Getting started

On one terminal:

```console
ros2 run mission_planner_srv service
```

and on another:

```console
ros2 run mission_planner_srv client
```

to get the main idea of what's going on.

Tested on amd64: foxy, galactic, and humble.

## Dependencies

* [swarm_interfaces](https://github.com/halehaka/interfaces_swarm)
