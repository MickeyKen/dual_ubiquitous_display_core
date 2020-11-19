#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time
import sys
from std_msgs.msg import Float64,Int16
import math
from std_msgs.msg import String, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospkg
import rosparam


spicyfood = False
def control_pantilt(pan1,tilt1,pan2,tilt2):
    head_pub = rospy.Publisher('/dynamixel_workbench_head/joint_trajectory', JointTrajectory, queue_size=100)
    jtp_msg = JointTrajectoryPoint()
    head_msg = JointTrajectory()
    head_msg.joint_names = [ "pantilt1_pan_joint", "pantilt1_tilt_joint", "pantilt2_pan_joint", "pantilt2_tilt_joint"]
    head_msg.header.stamp = rospy.Time.now()
    jtp_msg.positions = [pan1,tilt1, pan2,tilt2]
    jtp_msg.velocities = [0.5,0.5,0.5,0.5]
    jtp_msg.time_from_start = rospy.Duration.from_sec(0.00000002)
    head_msg.points.append(jtp_msg)
    head_pub.publish(head_msg)

if __name__ == '__main__':
    rospy.init_node('dual_ubiquitous_display_demo', anonymous=True)
    head_pub = rospy.Publisher('/dynamixel_workbench_head/joint_trajectory', JointTrajectory, queue_size=100)
    print ("****** Start Program ******")
    spicyfood = input("Do you use pantilt? True or False?")
    spicyfood = "True" 
    print ("set timer 2.0 (sec)")
    rospy.sleep(2.0)

    #if spicyfood == "True":
    control_pantilt(0.392699081699,0.0,0.392699081699,0.0)
    if spicyfood == "False":
        pass

    rospy.sleep(4.0)


    ### set left pantilt ###
    #if spicyfood == "True":
        #print ("push")
    control_pantilt(0.392699081699,0.0,0.392699081699,0.820304748437)
    if spicyfood == "False":
        pass
    rospy.sleep(4.0)
    rospy.set_param('left_projector/switch', True)

    ### set right pantilt ###
    #if spicyfood == "True":
    control_pantilt(0.392699081699,0.837758040957,0.392699081699, 0.8329516053199768)
    if spicyfood == "False":
        pass
    rospy.sleep(2.0)
    rospy.set_param('right_projector/switch', True)

    ### Terminate program ###
    rospy.sleep(5.0)
    rospy.set_param('left_projector/switch', False)
    rospy.set_param('right_projector/switch', False)
    rospy.sleep(2.0)
    if spicyfood == "True":
        control_pantilt(math.radians(22.5),0.0,math.radians(22.5),0.0)
    if spicyfood == "False":
        pass

    print ("****** End Program ******")
