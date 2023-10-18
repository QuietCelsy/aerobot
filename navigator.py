#!/usr/bin/env python3

# """
#  * File: offb_node.py
#  * Stack and tested in Gazebo Classic 9 SITL
# """

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool

current_state = State()

pose = PoseStamped()
pose.pose.position.z = 2

def state_cb(msg):
    global current_state
    current_state = msg

def goal_callback(msg):
    print('got pose')
    pose.pose.position.x = msg.pose.position.x
    pose.pose.position.y = msg.pose.position.y
    confirmation.data = False


if __name__ == "__main__":
    rospy.init_node("offb_node_py")



    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    goal_sub = rospy.Subscriber('drone_goal', PoseStamped, callback=goal_callback)

    confirm = rospy.Publisher('result', Bool, queue_size=10)

    confirmation = Bool()
    confirmation.data = False



    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
 
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        # print('kekw')

        local_pos_pub.publish(pose)

        msg = rospy.wait_for_message("/mavros/local_position/pose", PoseStamped)

        #print(pose.pose.position.x, msg.pose.position.x)
        #print(pose.pose.position.y, msg.pose.position.y)
        #print(pose.pose.position.z, msg.pose.position.z)

        if msg.pose.position.x - pose.pose.position.x > - 0.1 and msg.pose.position.x - pose.pose.position.x < 0.1:
            #print('1 true')
            if msg.pose.position.y - pose.pose.position.y > - 0.1 and msg.pose.position.y - pose.pose.position.y < 0.1:
                #print('2 true')
                if msg.pose.position.z - pose.pose.position.z > - 0.1 and msg.pose.position.z - pose.pose.position.z < 0.1:
                    #print('3 true')
                    # print('goal achived')
                    print(pose)
                    confirmation.data = True
                    confirm.publish(confirmation)
                else:
                    rate.sleep()
                    continue
