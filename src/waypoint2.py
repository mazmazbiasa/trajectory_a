#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State

FLIGHT_ALTITUDE = 2

current_state_uav0 = State()
current_state_uav1 = State()
current_state_uav2 = State()

pose_uav0 = PoseStamped()
pose_uav1 = PoseStamped()
pose_uav2 = PoseStamped()

# Subscriber Define
state_sub_uav0 = rospy.Subscriber("uav0/mavros/state", State, state_cb_uav0)
local_pos_pub_uav0 = rospy.Publisher("uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
arming_client_uav0 = rospy.ServiceProxy("uav0/mavros/cmd/arming", CommandBool)
land_client_uav0 = rospy.ServiceProxy("uav0/mavros/cmd/land", CommandTOL)
set_mode_client_uav0 = rospy.ServiceProxy("uav0/mavros/set_mode", SetMode)

# Define callback functions
def state_cb_uav0(msg1):
    global current_state_uav0
    current_state_uav0 = msg1

def state_cb_uav1(msg2):
    global current_state_uav1
    current_state_uav1 = msg2

def state_cb_uav2(msg3):
    global current_state_uav2
    current_state_uav2 = msg3

def follow(fmsg):
    global pose_uav0
    pose_uav0 = fmsg
    pose_uav0.pose.position.x = pose_uav0.pose.position.x + 1.0
    local_pos_pub_uav0.publish(pose_uav0)

def main():
    rospy.init_node("multidronecoba_node")
    nh_uav0 = rospy.NodeHandle()
    nh_uav1 = rospy.NodeHandle()
    nh_uav2 = rospy.NodeHandle()

    rate = rospy.Rate(40)  # 40 Hz

    while not (rospy.is_shutdown() or current_state_uav0.connected and current_state_uav1.connected and current_state_uav2.connected):
        rospy.loginfo("Connecting to FCU...")
        rospy.spinOnce()
        rate.sleep()

    pose_uav0.pose.position.x = 0
    pose_uav0.pose.position.y = 0
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE

    # Sending a few setpoints before starting
    for i in range(100):
        local_pos_pub_uav2.publish(pose_uav2)
        local_pos_pub_uav1.publish(pose_uav1)
        local_pos_pub_uav0.publish(pose_uav0)
        rospy.spinOnce()
        rate.sleep()

    rospy.loginfo("Multi UAV ready to go")

    # Waypoint 1
    for i in range(10 * 30):
        local_pos_pub_uav0.publish(pose_uav0)
        local_pos_pub_uav1.publish(pose_uav1)
        local_pos_pub_uav2.publish(pose_uav2)
        rospy.spinOnce()
        rate.sleep()

    rospy.loginfo("Waypoint 1 reached")

    # Waypoint 2
    pose_uav0.pose.position.x = 3
    for i in range(10 * 30):
        local_pos_pub_uav0.publish(pose_uav0)
        local_pos_pub_uav1.publish(pose_uav1)
        local_pos_pub_uav2.publish(pose_uav2)
        rospy.spinOnce()
        rate.sleep()

    rospy.loginfo("Waypoint 2 reached")

    # Waypoint 3
    pose_uav0.pose.position.y = 3
    for i in range(10 * 30):
        local_pos_pub_uav0.publish(pose_uav0)
        local_pos_pub_uav1.publish(pose_uav1)
        local_pos_pub_uav2.publish(pose_uav2)
        rospy.spinOnce()
        rate.sleep()

    rospy.loginfo("Waypoint 3 reached")

    # Waypoint 4
    pose_uav0.pose.position.x = 0
    for i in range(10 * 30):
        local_pos_pub_uav0.publish(pose_uav0)
        local_pos_pub_uav1.publish(pose_uav1)
        local_pos_pub_uav2.publish(pose_uav2)
        rospy.spinOnce()
        rate.sleep()

    rospy.loginfo("Waypoint 4 reached")

    # Land position
    pose_uav0.pose.position.y = 0
    for i in range(10 * 930):
        local_pos_pub_uav0.publish(pose_uav0)
        local_pos_pub_uav1.publish(pose_uav1)
        local_pos_pub_uav2.publish(pose_uav2)
        rospy.spinOnce()
        rate.sleep()

    rospy.loginfo("Landing")

if __name__ == "__main__":
    main()
