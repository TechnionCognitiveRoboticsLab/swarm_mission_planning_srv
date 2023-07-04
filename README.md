# swarm_mission_planning_srv
A ROS2 service that plans missions for swarms.

## Installation

This is assuming you already have Ubuntu 22.04, ROS2 Humble, and tensorflow

Download the packages
```console
mkdir -p mission_planning_ws/src
cd mission_planning_ws/src
git clone git@github.com:halehaka/swarm_mission_planning_srv.git
git clone git@github.com:halehaka/map_cover.git
git clone git@github.com:halehaka/interfaces_swarm.git
```

Build the workspace
```console
source /opt/ros/humble/setup.bash
cd ~/mission_planning_ws
colcon build
source  install/setup.bash
```

To have some maps to play with (note that this will download about 1.4GB of maps, so you can skip this), run
```console
mkdir -p ~/mission_planning_ws/landcover
cd ~/mission_planning_ws/landcover
wget https://landcover.ai.linuxpolska.com/download/landcover.ai.v1.zip
unzip landcover.ai.v1.zip
```

## Getting started

We have two services to run. 

The first one takes as input a map, a grid size, and some annotated polygons (optional) and outputs a grid with target detection probabilities for each cell.
To run this, open a terminal and run 
```console
source ~/mission_planning_ws/install/setup.bash
ros2 run map_cover predict_node
```
You will see some tensorflow warnings and errors - they are safe to ignore


In a second terimal, run the mission planning service, which takes as input a grid and initial locations for the drones, and outputs a plan
```console
source ~/mission_planning_ws/install/setup.bash
ros2 run mission_planner_srv service
```

Finally, to test the planning service, open another terminal and run:

```console
source ~/mission_planning_ws/install/setup.bash
ros2 run mission_planner_srv client ~/mission_planning_ws/landcover/images/M-33-7-A-d-2-3.tif
```
You can replace the image with any other map.
This first calls the map cover service to get a grid, and then the mission planning service with the grid to get a plan

## Dependencies

* [swarm_interfaces](https://github.com/halehaka/interfaces_swarm)
* [map_cover](https://github.com/halehaka/map_cover)
