#!/usr/bin/env python3
import os
import time

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import apriltag


class my_state:
    def __init__(self, board_found=False, board_num=-1, is_going_to_right=True):
        self.board_found = board_found
        self.board_num = board_num
        self.is_going_to_right = is_going_to_right
        self.checked = False

    def __str__(self):
        return "Boards found: %s\nBoard num: %d\nGoing rigt?: %s\nChecked?: %s" % (
            "Yes" if self.board_found else "No", self.board_num, "Yes" if self.is_going_to_right else "No",
            self.checked)


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = {}  # 3d point in real world space
imgpoints = {}  # 2d points in image plane.


def bgr_from_jpg(data):
    """ Returns an OpenCV BGR image from a string """
    s = np.fromstring(data, np.uint8)
    bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if bgr is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return bgr


def calib_test(img, mtx, dist):
    # undistort
    dst = cv2.undistort(img, mtx, dist)
    # crop the image
    # x, y, w, h = roi
    # dst = dst[y:y + h, x:x + w]

    cv2.imshow("test", img)
    cv2.imshow("test1", dst)
    cv2.waitKey(0)

    return dst


class MyNode():

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.img = None
        self.pub = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.sub_image = rospy.Subscriber("~image/compressed", CompressedImage, self.processImage, queue_size=1)
        self.gray = None
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)

    def detectTags(self):
        return self.detector.detect(self.gray)

    def processImage(self, image_msg):
        # print('processImage()')
        self.img = bgr_from_jpg(image_msg.data)
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    def step(self, is_going_to_right=True):
        msg = WheelsCmdStamped()
        rate = rospy.Rate(0.5)  # 1Hz
        rate2 = rospy.Rate(10)
        msg.vel_left = -0.2
        msg.vel_right = 0.2
        if is_going_to_right:
            msg.vel_left, msg.vel_right = msg.vel_right, msg.vel_left
        # rospy.loginfo("Publishing message")
        self.pub.publish(msg)
        rate2.sleep()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        # rospy.loginfo("Publishing message -")
        self.pub.publish(msg)
        rate.sleep()

    def is_chessboard_found(self, img):
        ret, corners = cv2.findChessboardCorners(self.gray, (7, 5), None)
        return ret == True

    def handle_img(self, img):
        ret, corners = cv2.findChessboardCorners(self.gray, (7, 5), None)
        if ret == True:
            all_x = [pair[0][0] for pair in corners]
            objpoints[tuple(corners.tobytes())] = objp
            cv2.cornerSubPix(self.gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints[tuple(corners.tobytes())] = corners
            cv2.drawChessboardCorners(img, (7, 5), corners, ret)
            return (min(all_x), max(all_x))
        return None

    def run(self):
        cw = True
        rate = rospy.Rate(1)  # 1Hz
        dir_changes = 0
        board = 0  # CCW from 0
        # cw, isVis, restor
        state = my_state(board_found=cw, board_num=board, is_going_to_right=True)
        while not rospy.is_shutdown():
            if (self.img is not None) and (self.gray is not None):
                if self.is_chessboard_found(self.img):
                    state.board_found = True
                    if not state.checked:
                        state.board_num += 1 if state.is_going_to_right == True else -1
                        state.checked = True
                        res = self.handle_img(self.img)
                        print(res)

                else:
                    state.board_found = False
                    if ((state.board_num <= -1) or (state.board_num >= 1)) and state.checked:
                        state.is_going_to_right = not state.is_going_to_right
                        state.checked = False
                        dir_changes += 1
                    elif (state.board_num == 0) and (dir_changes == 2):
                        dir_changes += 1
                    elif state.board_num == 0:
                        state.checked = False
                print(state)
                print(dir_changes)
                self.step(state.is_going_to_right)
                cv2.imshow('img', self.img)
                cv2.waitKey(500)
                if dir_changes >= 3:
                    break
        cv2.destroyAllWindows()
        self.sub_image.unregister()
        print(len(list(objpoints.values())))
        print(len(objpoints.values()), len(imgpoints.values()))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(list(objpoints.values()), list(imgpoints.values()),
                                                           self.gray.shape[::-1],
                                                           None, None)
        print('cv2.calibrateCamera: ')
        print('mtx = ', mtx)
        print('dist = ', dist)
        print('rvecs = ', rvecs)
        print('tvecs = ', tvecs)
        h = 480
        w = 640
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0.1, (w, h))
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
        dst = cv2.remap(self.img, mapx, mapy, cv2.INTER_LINEAR)
        cv2.imwrite('/tmp/calibresult.png', dst)

        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(list(objpoints.values())[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(list(imgpoints.values())[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("total error: ", mean_error / len(objpoints))
        rospy.signal_shutdown('calibration done')

        calib_test(self.img, mtx, dist)


start = time.time()
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='autocalib_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
print("finished in " + str(time.time() - start))
