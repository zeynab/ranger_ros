ROS adapter for the EPFL Ranger robot
=====================================

This ROS package provides:

- a set of launch files suitable to create maps or drive around the robot
  (assuming a 3D depth sensor)
- a node (`ros_interface`) that expose standard ROS topics for the robot
  (odometry as `odom` and the TF transformation for `base_link`)


![ROS mapping with the Ranger](ranger_mapping.png "gmapping on the Ranger, viewed in RViz")

Dependencies
------------

- [`ranger_description`](https://github.com/severin-lemaignan/ranger_description)
- [`pyranger`](https://github.com/chili-epfl/pyranger)

+ ROS 2D SLAM and navigation tools (`gmapping`, `amcl`, `move_base`...)

Technical checks
----------------

- Checking the odometry is ok: run `rqt_plot` + Ranger's `teleop`, plot `/cmd_vel/linear/x`, `/cmd_vel/angular/z`, `/odom/twist/linear/x` and `/odom/twist/linear/z` and check everyone match.
