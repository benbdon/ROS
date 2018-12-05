# homework-1-f2017-benbdon

## Functionality
### The package `inverseKinematics` creates a visualization of a 2R planar robot. The end-effector of the robot follows a path in the XY plane described by by a x_desired(t) & y_desired(t). Additionally a 1 second history of previous locations are shown on the visualization with point markers.



## Publish/Subscribe Topics
### +`/space/joint_states` hosts the two joint angles (technically there are three since one of them is the fixed joint connecting the base_link to the first link)
### +`tf_static` contains the all the coordinate frames and their relative positions to one another



## Parameters
### +`/robot_description` the URDF model of the 2R robot



## Key Nodes
### +`/space/ben` publishes joint states using inverse kinematics of a 2R robot detailed in Chapter 6 of "Modern Robotics" by Kevin Lynch and Frank C Park
### +`/space/robot_state_publisher` takes the joint angles and publishes the 3D poses of the robotic links, using a kinmatic tree model of the robot.
### +`/space/coord_publisher` publishes coordinates for the markers in `rviz` to using the a transformation from the base_link to the end_effector link.



## Launch File
### +Loads the URDF into the parameter server
### +Launches instance of node, `joint_publisher`, named `Ben`
### +Launches instance of node, `robot_state_publisher`, named 'robot_state publisher'
### +Launches rviz and loads its configuration file
