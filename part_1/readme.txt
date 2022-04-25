READ ME for ENPM 661 Project 3 Phase 2
Qamar Syed

Environment: Python 3.7 with OpenCV, NumPy, PriorityQueue, and Math

Steps:

1. Initialize values:
variables for start position, goal position, wheel rpm, and clearance are located at the top of the document and named accordingly
(rpm1 is left and rpm2 is right wheel rpm)
(x_start, y_start, angle_start, x_goal, y_goal, and clearance are input as well, angle_start is in degrees)

2. Run script through terminal or in IDE in directory of the script

3. Video output will be stored in the current directory


Notes:
Just used the given cost formula and an 8 action set to make the script, although the paths don't look very 'curved' in the video, they do follow the differential drive equations, it is just due to the small robot size, if the robot was larger these would appear much more curved
