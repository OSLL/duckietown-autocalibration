#!/usr/bin/env python3
import os
import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


def bgr_from_jpg(data):
    """ Returns an OpenCV BGR image from a string """
    s = np.fromstring(data, np.uint8)
    bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if bgr is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return bgr

class MyNode():

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.img = None
        self.pub = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.sub_image = rospy.Subscriber("~image/compressed", CompressedImage, self.processImage, queue_size=1)

    def processImage(self, image_msg):
        print('processImage()')
        self.img = bgr_from_jpg(image_msg.data)
    
    def step(self, cw = True):
        msg = WheelsCmdStamped()
        rate = rospy.Rate(10) # 1Hz
        msg.vel_left = -0.0225
        msg.vel_right = 0.0225
        if cw:
            msg.vel_left, msg.vel_right = msg.vel_right, msg.vel_left
        rospy.loginfo("Publishing message")
        self.pub.publish(msg)
        rate.sleep()
        msg.vel_left =  0.0
        msg.vel_right = 0.0
        rospy.loginfo("Publishing message -")
        self.pub.publish(msg)
        rate.sleep()

    def handle_img(self, img):
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(self.gray, (7,5),None)
        if ret == True:
            all_x = [pair[0][0] for pair in corners]
            objpoints.append(objp)
            cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(img, (7,5), corners,ret)
            return (min(all_x), max(all_x))
        return None

    def fsm(self, state, board):
        if board > 1:
            return (True,  False, True ), board-1
        if board < 0:
            return (False,  False, True ), board+1
        rules = {
            # cw,   isVis, restor
            (False, True,  False): ((False, True,  False), board  ),  # normal CCW
            (True,  True,  False): ((True,  True,  False), board  ),  # normal CW
            (False, False, False): ((False, False, True ), board+1),  # insiv when CCW -> go CW back
            (True,  False, False): ((True,  False, True ), board-1),  # insiv when CW -> go CCW back
            
            # (False, False, False): ((True,  False, True ), !)  # insiv when CCW -> go CW back
            # (True,  False, False): ((False, False, True ), !)  # insiv when CW -> go CCW back
            (True,  False, True ): ((True,  False, True ), board  ),  # normal CW restoration
            (False, False, True ): ((False, False, True ), board  ),  # normal CCW restoration
            (True,  True,  True ): ((True,  True,  False), board  ),  # restores, now normal CW movement
            (False, True,  True ): ((False, True,  False), board  )  # restores, now normal CCW movement
        }
        return rules[state][0], rules[state][1]

    def run(self):
        cw = True
        rate = rospy.Rate(1.5) # 1Hz
        dir_changes = 0
        board = 0 # CCW from 0
        # cw, isVis, restor
        state = (cw, True, False)
        while not rospy.is_shutdown():
            if self.img is not None:
                img = self.img
                res = self.handle_img(img)
                print(res)
                print(state)
                if res is None:
                    state = (state[0], False, state[2])
                else:
                    state = (state[0], True, state[2])
                new_state, board = self.fsm(state, board)
                if state[0] != new_state[0]:
                    dir_changes += 1
                state = new_state    
                print(state)
                self.step(state[0])
                cv2.imshow('img', img)
                cv2.waitKey(500)
                if dir_changes >=2:
                    break
        cv2.destroyAllWindows()
        self.sub_image.unregister()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, self.gray.shape[::-1],None,None)
        print('cv2.calibrateCamera: ')
        print('mtx = ', mtx)
        print('dist = ', dist)
        print('rvecs = ', rvecs)
        print('tvecs = ', tvecs)
        h = 480
        w = 640
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0.1,(w,h))
        mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
        dst = cv2.remap(self.img,mapx,mapy,cv2.INTER_LINEAR)
        cv2.imwrite('/tmp/calibresult.png',dst)

        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print("total error: ", mean_error/len(objpoints))
        rospy.signal_shutdown('calibration done')


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='autocalib_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
