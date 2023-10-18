#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
import numpy as np
import math

class Nav():
    def __init__(self):
        rospy.init_node('navigation')
        self.rate = rospy.Rate(20)
        self.goal = PoseStamped()
        self.goal_pub = rospy.Publisher('drone_goal', PoseStamped, queue_size=10)
        self.empty_spaces = Int32MultiArray()
        self.chosen_space = Int32MultiArray()

    def send_goal(self):
        self.goal.pose.position.x = self.x
        self.goal.pose.position.y = self.y
        self.goal.pose.orientation.w = 1.0
        self.goal_pub.publish(self.goal)

    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges, float)
        indexes = np.where(self.ranges == math.inf)
        print(indexes)
        for i in range(len(indexes[0])):
            try:
                if indexes[0][i + 1] - indexes[0][i] > 10:
                    print('space found!')
                    self.empty_spaces.data.append(indexes[0][i + 1])
                    self.empty_spaces.data.append(indexes[0][i])
            except:
                continue
        try:
            if indexes[0][0] != 0 and indexes[0][-1] != 360:
                print('space found!')
                self.empty_spaces.data.append(indexes[0][0])
                self.empty_spaces.data.append(indexes[0][-1])
        except Exception as error:
            print(error)
        counter = 0
        self.grouped_spaces = [self.empty_spaces.data[n:n+2] for n in range(0, len(self.empty_spaces.data), 2)]
        print(self.empty_spaces)
        self.chosen_space.data = self.grouped_spaces[counter]
        print(self.chosen_space.data)
        if len(self.grouped_spaces) > 1:
            self.empty_spaces.data.insert(0, self.chosen_space.data[0])
            self.empty_spaces.data.insert(0, self.chosen_space.data[1])
        print('scan finished')

    def vector_calculation(self, ranges):
        try:
            print(self.chosen_space.data[1], self.chosen_space.data[0])

            # print(ranges[self.chosen_space.data[0] - 1], ranges[self.chosen_space.data[1] + 1])
            verctor_lengh = math.sqrt(ranges[self.chosen_space.data[0] - 1]**2 + ranges[self.chosen_space.data[1] + 1]**2 + 2 
                                               * ranges[self.chosen_space.data[1] + 1] * ranges[self.chosen_space.data[0] - 1]
                                                * math.cos((math.radians(self.chosen_space.data[0] + self.chosen_space.data[1])/2)))
            self.x = verctor_lengh  * math.sin((math.radians(self.chosen_space.data[1] + self.chosen_space.data[0])/2))
            self.y = verctor_lengh  * math.cos((math.radians(self.chosen_space.data[1] + self.chosen_space.data[0])/2))
            # print(self.odom_y, self.odom_x, verctor_lengh)
            print("x --" ,self.x , "y -- ", self.y)
            # print(self.previous_x, self.previous_y)
        except Exception as error:
            print('Error in vector_calculation', error)
        print('vector finished')


    def spin(self):
        print('spin started - drive')
        self.scan_callback(rospy.wait_for_message('/scan', LaserScan))
        self.vector_calculation(self.ranges)
        self.send_goal()
        while True:
            result = rospy.wait_for_message('result', Bool)
            if result.data == True:
                self.rate.sleep()

if __name__ == '__main__':
    nav = Nav()
    while not rospy.is_shutdown():
        nav.spin()
        nav.rate.sleep()