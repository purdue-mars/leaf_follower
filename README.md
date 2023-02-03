# leaf_follower
Using GelSight and UR5e to follow a leaf

## Dependencies
- [gelsight_wedge_controller](https://github.com/purdue-mars/gelsight_wedge_controller)
  - Any gripper with velocity control should do, although there is specific code for the wedge gripper that enforces joint limits in the grasp controller and launch file in `follower.launch`
- UR5e control
  - [UR5 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
  - [Twist controller](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md#twist_controller)
  - Controller itself just requires a http://wiki.ros.org/twist_controller, so if your arm can implement cartesian servoing, it should work
- [Gelsight ROS](https://github.com/purdue-mars/gelsight-ros)
  - Grasp control: uses the diff image to compute a contact area estimate for force control
  - Following control: uses PCA and estimated pose of object within gelsight pad for control
  
## Get started

1. Ensure dependencies and robot/gripper is setup correctly, build + source
2. Run `roslaunch mars_leaf_follower follower.launch` to run gelsight feedback, gripper control, and ur5e controllers
3. Tune Kp,Kd gains for both grasp controller and follower control
4. Run `rosrun mars_leaf_follower grasp_control_node` for grasp control (make sure leaf/cable is within grip)
5. Run `rosrun mars_leaf_follower follower_node` for following
