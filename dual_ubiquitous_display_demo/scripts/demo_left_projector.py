#!/usr/bin/env python
import cv2
import numpy as np
from std_msgs.msg import Bool
import rospy
import time
import sys
import math
import rospkg
import rosparam
r = rospkg.RosPack()
path = r.get_path('dual_ubiquitous_display_projector')

images = ["png","jpg","jpeg"]
videos = ["mov","mp4"]

if __name__ == '__main__':
    rospy.init_node('demo_left_projector_imshow', anonymous=True)

    pts_src = np.array([[0,0], [1919,0], [1919, 1079],[0,1079]],np.float32)
    pts1_org = np.array([[-950, 2100],[470, 2100],[200, 1040],[-660, 1040]])

    h, status = cv2.findHomography(pts_src, pts1_org)
    h = np.linalg.inv(h)

    new_pts_dst = np.array([[0, 0],[0, 0],[0, 0],[0, 0]],np.float32)

    f = np.float32([[-600.0, 1570.0 + 225.0, 1.0]])
    f = f.T
    f = np.dot(h, f)
    f = f / f[2][0]
    new_pts_dst[0][0] = f[0][0]
    new_pts_dst[0][1] = f[1][0]

    s = np.float32([[200.0, 1570.0 + 225.0, 1.0]])
    s = s.T
    s = np.dot(h, s)
    s = s / s[2][0]
    new_pts_dst[1][0] = s[0][0]
    new_pts_dst[1][1] = s[1][0]

    t = np.float32([[-600.0, 1570.0 - 225.0, 1.0]])
    t = t.T
    t = np.dot(h, t)
    t = t / t[2][0]
    new_pts_dst[3][0] = t[0][0]
    new_pts_dst[3][1] = t[1][0]

    fo = np.float32([[200.0, 1570.0 - 225.0, 1.0]])
    fo = fo.T
    fo = np.dot(h, fo)
    fo = fo / fo[2][0]
    new_pts_dst[2][0] = fo[0][0]
    new_pts_dst[2][1] = fo[1][0]

    M = cv2.getPerspectiveTransform(pts_src,new_pts_dst)

    rospy.set_param('left_projector/switch', False)
    rospy.set_param("left_projector/file_name", "cc_7F_green_180.png")

    while not rospy.is_shutdown():
        sw = rospy.get_param("left_projector/switch")

        if sw:
            file_name = rospy.get_param("left_projector/file_name")
            ext = file_name.split('.')[1]

            if ext in images:
                img = cv2.imread(path + '/Images/' + file_name)
                h = 1920
                w = 1080
                img = cv2.resize(img,(h,w))
                warp = cv2.warpPerspective(img, M, (1920,1080))
                cv2.namedWindow('screen2', cv2.WINDOW_NORMAL)
                cv2.setWindowProperty('screen2', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                # cv2.moveWindow('screen',1940,0)
                cv2.imshow('screen2', warp)
                cv2.waitKey(1)
            elif ext in videos:
                cap = cv2.VideoCapture(path + '/Images/' + file_name)
                while(cap.isOpened()):
                    ret, frame = cap.read()
                    if not ret:
                        break
                    else:
                        last_frame = frame
                    warp = cv2.warpPerspective(frame, M, (1920,1080))
                    cv2.imshow("screen2", warp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                warp = cv2.warpPerspective(last_frame, M, (1920,1080))
                cv2.imshow("screen2", warp)
                cv2.waitKey(5000)
            else:
                pass
        else:
            black_img = np.zeros((1080,1920,3),np.uint8)
            cv2.namedWindow('screen2', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('screen2', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            # cv2.moveWindow('screen',1940,0)
            cv2.imshow('screen2', black_img)
            cv2.waitKey(1)
