# semantic_auto_expl
==============

This repository holds packages for autonomous exploration and semantic mapping for both simulated and real Robotino.

# Requirements

ROS packages required:
* squirrel_common
* squirrel_nav
* squirrel_robotino
* squirrel_robotino_arm
* gmapping
* move_base
* rviz

other libraries required:
* OpenCV
* Point Cloud Library (PCL)

# Features

* Autonomous Exploration
* Open Door Detecion
* Changing Door State Detection
* Dangerous Area Detection

# Usage

To start a simulated demo for autonomous exploration and semantic mapping in an example building execute

```bash
roslaunch semantic_mapping start_semantic_expl_sim.launch robot:=ipa-robotino
```

When using a real robotino where no simulation is needed execute

```bash
roslaunch semantic_mapping start_semantic_expl_real.launch robot:=ipa-robotino
```

The launch file also starts rviz for map and semantic info visualization. Semantic infos are visualized using
rviz markers:
* The current autonomous navigation target is visualized using a yellow circle
* Open doors/doorways are visualized using green circles
* Closed doors are visualized using blue circles
* Dangerous areas are visualized using red areas

If better performance is needed there is also a launch file without gazebo visualization (rviz only):

```bash
roslaunch semantic_mapping start_semantic_expl_with_sim_rviz_only.launch robot:=ipa-robotino
```

The single algorithm components can be enabled/disabled by changing the corresponding parameters to true/false, e.g.

```bash
<arg name="auto_exploration" default="false" />
<arg name="open_door_detection" default="true" />
<arg name="dangerous_area_detection" default="true" />
```

Robotino params such as the kinect height need to be adjusted in the header files of the single components every time something is changed in the robot setup.




