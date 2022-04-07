#!/usr/bin/env python3
import os
import rospy
import tf
import math
import time
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import AprilTagDetection
from sensor_msgs.msg import CompressedImage


class MyNode():

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.img = None
        self.pub = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.sub_tags = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTags, queue_size=1)
        self.yaw = None
        self.vel = 0.23
        self.k = 1.0
        self.truthn = 0
        self.centre = False
        self.start_process_tags = False
        self.delta = 0

    def processTags(self, tags_msg):
        self.start_process_tags = True
        # print('processTags()')
        minz = None
        if len(tags_msg.detections) == 0:
            self.delta += 1
        else:
            self.delta = 0
        for tag in tags_msg.detections:
            print(f"x = %f y = %f z = %f w = %f" % tuple(x / (2 * math.pi) * 360 for x in (
            tag.transform.rotation.x, tag.transform.rotation.y, tag.transform.rotation.z, tag.transform.rotation.w)))
            (r, p, y) = tf.transformations.euler_from_quaternion(
                [tag.transform.rotation.x, tag.transform.rotation.y, tag.transform.rotation.z,
                 tag.transform.rotation.w])
            # print (y)
            if tag.transform.rotation.z < 0.20:
                self.stop = True
            if minz is None or tag.transform.rotation.z < minz:
                self.yaw = y
            if abs(self.yaw) < 0.15:
                self.centre = True
            # print (r, p, y)

    def sendVel(self, vel):
        msg = WheelsCmdStamped()
        msg.vel_left = vel
        msg.vel_right = vel * self.k
        rospy.loginfo("Publishing message: {} {}".format(msg.vel_left, msg.vel_right))
        self.pub.publish(msg)

    def processImage(self, image_msg):
        # print('processImage()')
        self.img = bgr_from_jpg(image_msg.data)

    def onStart(self):
        msg = WheelsCmdStamped()
        print('moving')
        self.centre = False
        while not rospy.is_shutdown():
            if not self.start_process_tags:
                continue
            msg.vel_left = -0.1
            msg.vel_right = 0.1
            self.pub.publish(msg)
            time.sleep(0.25)
            msg.vel_left = 0.00
            msg.vel_right = 0.00
            self.pub.publish(msg)
            time.sleep(0.5)
            if self.centre:
                return

    def findWay(self):
        self.onStart()
        while self.delta < 100:
            self.sendVel(0.1)

    def run(self):
        # rate = rospy.Rate(0.6) # 1Hz
        self.onStart()
        while not rospy.is_shutdown():
            print(self.yaw)
            if self.yaw is None:
                continue
            old_yaw = self.yaw
            self.sendVel(self.vel)
            start_time = time.time()
            time.sleep(3.0)
            self.stop = False
            for i in range(10):
                if self.stop:
                    break
                time.sleep(0.05)
            # rate.sleep()

            self.sendVel(0)
            end_time = time.time()
            time.sleep(3)
            # rate.sleep()
            cur_yaw = self.yaw
            print('dtheta = ', math.degrees(cur_yaw - old_yaw))
            dtheta = cur_yaw - old_yaw
            dtheta *= -1.0
            l = 0.1
            r = 0.066
            dt = end_time - start_time
            omegal = self.vel
            omegar = self.vel * self.k
            k = (omegal + ((dtheta * l) / (dt * r))) / omegar
            print('cur k = ', k)
            gain_new = (1.0 - k) / 2.0
            print('gain_new = ', gain_new)
            if abs(gain_new) < 0.05:
                self.truthn += 1
                print('self.truthn = ', self.truthn)
            else:
                self.truthn = 0
                print('self.truthn = ', self.truthn)
                gain_old = 1.0 - self.k
                k = 1.0 - (gain_new + gain_old)
                self.k = k
            if self.truthn >= 3:
                self.findWay()
                rospy.signal_shutdown('calibration done')
            print('self.k = ', self.k)
            self.sendVel(-self.vel)
            time.sleep(3.1)
            self.sendVel(0)
            time.sleep(3)


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='autocalib_node')
    # run node
    time.sleep(1)
    node.run()
    # keep spinning
    rospy.spin()
