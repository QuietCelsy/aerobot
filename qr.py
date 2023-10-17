#!/usr/bin/env python3

import cv2
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import rospy 
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

class QR_reader(object):
    def __init__(self):
        rospy.init_node('QR')

        self.goal = PoseStamped()

        self.quatr = 0

        self.goal.pose.position.x = 1
        self.goal.pose.position.y = 1
        self.goal.pose.position.z = 2.2
        self.goal.pose.orientation.w = 1

        self.q = quaternion_from_euler(0.0, 0.0, 1.5707)

        # self.confirm = rospy.Subscriber('result', Bool, self.result)

        self.pub_key = False

        self.image = Image()
        self.bridge = CvBridge()

        self.front_cam_sub = rospy.Subscriber("/r200/image_raw", Image, callback=self.front_read)
        self.low_cam_sub = rospy.Subscriber("/iris_rplidar/usb_cam/image_raw", Image, callback=self.low_read)

        self.pos_pub = rospy.Publisher("drone_goal", PoseStamped, queue_size=10)

        while not rospy.is_shutdown():
            print(self.pos_pub.get_num_connections())
            if self.pos_pub.get_num_connections() == 0:
                continue
            else:
                break
        self.pos_pub.publish(self.goal)
        self.qcd = cv2.QRCodeDetector()
        self.rate = rospy.Rate(20)

        # print(cv2.__version__)

    def result(self, msg):
        self.result = msg.data

    def state_cb(self ,msg):
        self.current_state = State()
        self.current_state = msg

    def front_read(self, img):
        frame = self.bridge.imgmsg_to_cv2(img)
        try:
            self.front_retval, decoded_info, points, straight_qrcode = self.qcd.detectAndDecodeMulti(frame)
            print('front -',self.front_retval ,decoded_info)
            if self.front_retval == True:
                self.front_qr = int(decoded_info[0])
        except Exception as e:
            print(e)

    def low_read(self, img):
        frame = self.bridge.imgmsg_to_cv2(img)
        try:
            self.low_retval, decoded_info, points, straight_qrcode = self.qcd.detectAndDecodeMulti(frame)
            print('low   -',self.low_retval ,decoded_info)
            if self.low_retval == True:
                self.low_qr = int(decoded_info[0])
        except Exception as e:
            print(e)

    def read(self):
        while True:
            try:
                self.low_qr
                self.front_qr
                print('in loop')
            except:
                continue
            else:
                print('kekw')
                break
        while True:
            msg =rospy.wait_for_message('result', Bool)
            print(msg.data)
            self.pos_pub.publish(self.goal)
            if msg.data != True:
                continue
            else:
                break
        if self.low_qr == self.front_qr and self.low_retval == True and self.front_retval == True:
            print('same qr')
            self.goal.pose.position.x += 5
        else:
            print('not same qr')
            try:
                # self.quatr += math.radians(90)
                # self.q = quaternion_from_euler(0.0, 0.0, self.quatr)
                # self.goal.pose.orientation = self.q
                self.goal.pose.position.x == 15
                print(self.goal)
            except Exception as e:
                print(e)
        time.sleep(3)
            
            
if __name__ == "__main__":
    qr_reader = QR_reader()
    while not rospy.is_shutdown():
        try:
            qr_reader.read()
            time.sleep(0.5)
        except KeyboardInterrupt as e:
            break