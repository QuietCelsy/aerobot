#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool

rospy.init_node('node', anonymous=True)
pub = rospy.Publisher('drone_goal', PoseStamped, queue_size=10)
rate = rospy.Rate(20)

pose = PoseStamped()

pose.pose.position.x = 5
pose.pose.position.y = 10
pose.pose.position.z = 3

while not rospy.is_shutdown():
    pub.publish(pose)
