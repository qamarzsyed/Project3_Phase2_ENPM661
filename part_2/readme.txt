READ ME for ENPM661 Proj3 Phase 2 ROS Simulation

Environment: ROS Noetic with Ubuntu 20.04

Instructions:

1. Place .launch and script files into appropriate folders in catkin_ws
(proj3.py was renamed from original name proj3 to prevent name overlap with other script file, should be renamed when replaced into catkin)
2. Place part 1 file in python site-packages folder
(this script gets the results of the a star algorithm from the part 1 script)

2. Run gazebo and spawn turtlebot in corresponding location to selected start point and angle in script

3. Run package with roslaunch [folder entered inside] proj3.launch


Notes:
Had a real difficult time with thi one as this is first semester and am new to ROS and Gazebo and although we were given instructions for Rviz I couldn't really figure out how to work everything in Gazebo. I could open the gazebo world but couldn't figure out how to spawn_model a turtlebot so I just attached an image of the open gazebo world. I ran the package in Rviz and it calculated the path and moved the robot, but it didn't seem to match the path that it should have. The script just takes the given vertical and angular velocity for every node, sends that to the turtlebot and waits a second before sending the velocity values for the next node. Since the determined time value was one second in the original python script and a new angular and linear velocity was calculated for every path between nodes, this should've been able to replicate the path from start to goal but it didn't. The turtlebot either moved in straight lines or just spun in circles. I'm not exactly sure how to fix this issue or anything and wasn't able to find enough resources on Gazebo to figure out that part (I could find stuff that instructed on how to create a URDF robot from scratch and spawn it but im sure there should be some way to directly insert a turtlebot).
