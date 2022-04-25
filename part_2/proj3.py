#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import math
import proj3_phase2_qamar_syed

# move set for rpm
# 0 = nothing yet
# 1 = left 0 right 1
# 2 = left 0 right 2
# 3 = left 1 right 0
# 4 = left 1 right 1
# 5 = left 1 right 2
# 6 = left 2 right 0
# 7 = left 2 right 1
# 8 = left 2 right 2

rpm1 = proj3_phase2_qamar_syed.rpm1
rpm2 = proj3_phase2_qamar_syed.rpm2
l = proj3_phase2_qamar_syed.l
r = proj3_phase2_qamar_syed.radius

def move_to_velocity(move_num):
    if move_num == 1:
        left = 0
        right = rpm1
    if move_num == 2:
        left = 0
        right = rpm2
    if move_num == 3:
        left = rpm1
        right = 0
    if move_num == 4:
        left = rpm1
        right = rpm1
    if move_num==5:
        left = rpm1
        right = rpm2
    if move_num==6:
        left = rpm2
        right = 0
    if move_num==7:
        left = rpm2
        right = rpm1
    if move_num==8:
        left = rpm2
        right = rpm2

    left = left*math.pi/30
    right = right*math.pi/30

    angular = (right-left)/l
    linear = 0.5*r*(left+right)
    return angular, linear

def proj3():
    goal = proj3_phase2_qamar_syed.a_star()
    path = goal.gen_path()
    velocity_list = []
    for node in path:
        moveID = node.moveID
        if moveID == 0:
            continue
        else:
            angular, linear = move_to_velocity(moveID)
            velocity_list.append((angular, linear))
            print(angular, linear)


    msg = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('robot_talker', anonymous=True)
    while not rospy.is_shutdown():
        for vel in velocity_list:
            ang, lin = vel
            msg.angular.z = ang
            msg.linear.x = lin
            # buff='my current time is %s" %rospy.get_time()
            pub.publish(msg)
            time.sleep(1)
        msg.angular.z = 0
        msg.linear.x = 0
        break



if __name__ == '__main__':
    proj3()
