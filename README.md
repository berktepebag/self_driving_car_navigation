# Self Driving RC car Navigation Package

Used for localization of the self driving RC car. 

### Launch files
1. localize_bag.launch is for tuning AMCL parameters with a rosbag file.
2. localize.launch is for localizing the self driving RC car with AMCL.
3. move_base.launch is for moving the car with TEB local planner. (NOT Complete!)

### Nodes
1. trajectory_utilities can save or load and publish paths generated with hector_trajectory_server node that can be called with "rosrun hector_trajectory_server hector_trajectory_server". 