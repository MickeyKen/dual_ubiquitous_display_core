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
    print ("set timer 2.0 (sec)")
    rospy.sleep(2.0)

    #if spicyfood == "True":
    control_pantilt(math.radians(22.5),0.0,math.radians(22.5),0.0)
    rospy.sleep(2.0)
    control_pantilt(math.radians(22.5)+math.radians(45),-math.radians(30),math.radians(22.5)+math.radians(45),-math.radians(30))
    rospy.sleep(3.0)
    control_pantilt(math.radians(22.5)-math.radians(45),math.radians(35),math.radians(22.5)+math.radians(45),math.radians(35))
    rospy.sleep(4.0)
    control_pantilt(math.radians(22.5),0.0,math.radians(22.5),0.0)
