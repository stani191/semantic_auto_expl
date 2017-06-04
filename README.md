# semantic_auto_expl
==============

This repository holds packages for autonomous exploration and semantic mapping for Robotino.

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

# Usage

To start a simulated demo for autonomous exploration and semantic mapping in an example building execute

```bash
roslaunch semantic_mapping start_semantic_expl_with_sim.launch robot:=ipa-robotino
```

The launch file also starts rviz for map and semantic info visualization. Semantic infos are visualized using
rviz markers:
* The current autonomous navigation target is visualized using a yellow circle
* Open doors/doorways are visualized using green circles

If better performance is needed there is also a launch file without gazebo visualization (rviz only):

```bash
roslaunch semantic_mapping start_semantic_expl_with_sim_rviz_only.launch robot:=ipa-robotino
```





