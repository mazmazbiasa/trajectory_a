#!/usr/bin/env python
import rospy
import os
import rospy
import rospkg
import subprocess
import roslaunch
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

# global variable
latitude = 0.0
longitude = 0.0
height = 2.0
current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg
    # print("callback")


def setOffboardMode():
    rospy.wait_for_service('/uav0/mavros/set_mode')
    rospy.wait_for_service('/uav1/mavros/set_mode')
    rospy.wait_for_service('/uav2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy(
            '/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='STABILIZED')  # return true or false
        
        flightModeService = rospy.ServiceProxy(
            '/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='STABILIZE')  # return true or false
        
        flightModeService = rospy.ServiceProxy(
            '/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='STABILIZE')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)


def setHoldMode():
    rospy.wait_for_service('/uav0/mavros/set_mode')
    rospy.wait_for_service('/uav1/mavros/set_mode')
    rospy.wait_for_service('/uav2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy(
            '/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='LOITER')
        flightModeService = rospy.ServiceProxy(
            '/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='LOITER')
        flightModeService = rospy.ServiceProxy(
            '/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='LOITER')
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. STABILIZE Mode could not be set. Check that GPS is enabled" %e)


def setLandMode():
    rospy.wait_for_service('/uav0/mavros/cmd/land')
    rospy.wait_for_service('/uav1/mavros/cmd/land')
    rospy.wait_for_service('/uav2/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy(
            '/uav0/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude=0, latitude=0,
                                longitude=0, min_pitch=0, yaw=0)
        landService = rospy.ServiceProxy(
            '/uav1/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude=0, latitude=0,
                                longitude=0, min_pitch=0, yaw=0)
        landService = rospy.ServiceProxy(
            '/uav2/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude=0, latitude=0,
                                longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("service land call failed: %s. The vehicle cannot land " %e)


def setArm():
    rospy.wait_for_service('/uav0/mavros/cmd/arming')
    rospy.wait_for_service('/uav1/mavros/cmd/arming')
    rospy.wait_for_service('/uav2/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy(
            '/uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
        armService = rospy.ServiceProxy(
            '/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
        armService = rospy.ServiceProxy(
            '/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" %e)


def setDisarm():
    rospy.wait_for_service('/uav0/mavros/cmd/arming')
    rospy.wait_for_service('/uav1/mavros/cmd/arming')
    rospy.wait_for_service('/uav2/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy(
            '/uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
        armService = rospy.ServiceProxy(
            '/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
        armService = rospy.ServiceProxy(
            '/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" %e)


def setTakeoffMode():
    rospy.wait_for_service('/uav0/mavros/set_mode')
    rospy.wait_for_service('/uav1/mavros/set_mode')
    rospy.wait_for_service('/uav2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy(
            '/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='AUTO.TAKEOFF')
        
        flightModeService = rospy.ServiceProxy(
            '/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='AUTO.TAKEOFF')
        
        flightModeService = rospy.ServiceProxy(
            '/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='AUTO.TAKEOFF')
    
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" %e)


##def setTakeoffMode():
    ##rospy.wait_for_service('/uav0/mavros/set_mode')
    ##rospy.wait_for_service('/uav1/mavros/cmd/takeoff')
    ##rospy.wait_for_service('/uav2/mavros/cmd/takeoff')
    ##try:
        ##flightModeService = rospy.ServiceProxy(
            ##'/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        ##isModeChanged = flightModeService(custom_mode='AUTO.TAKEOFF')
        
        ##takeoffService = rospy.ServiceProxy(
            ##'/uav1/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        ##takeoffService(altitude=height, latitude=0, longitude=0, min_pitch=0, yaw=0)
        
        ##takeoffService = rospy.ServiceProxy(
        ##'/uav2/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        ##takeoffService(altitude=height, latitude=0, longitude=0, min_pitch=0, yaw=0)
    
    #except rospy.ServiceException as e:
        #print("Service takeoff call failed: %s" %e)


def startWaypoint():
    rospy.wait_for_service('/uav0/mavros/set_mode')
    rospy.wait_for_service('/uav1/mavros/set_mode')
    rospy.wait_for_service('/uav2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy(
            '/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='OFFBOARD')  # return true or false
        
        flightModeService = rospy.ServiceProxy(
            '/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='OFFBOARD')  # return true or false
        
        flightModeService = rospy.ServiceProxy(
            '/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(
            custom_mode='OFFBOARD')  # return true or false
    
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)
    

##def startWaypoint():'/uav2/mavros/cmd/takeoff'
    ##rospy.wait_for_service('/uav0/mavros/set_mode')
    ##rospy.wait_for_service('/uav1/mavros/set_mode')
    ##rospy.wait_for_service('/uav2/mavros/set_mode')
    ##try:
        ##flightModeService = rospy.ServiceProxy(
            ##'/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        ##isModeChanged = flightModeService(
            ##custom_mode='OFFBOARD')  # return true or false
        
        ##flightModeService = rospy.ServiceProxy(
            ##'/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
        ##isModeChanged = flightModeService(
            ##custom_mode='GUIDED')  # return true or false
        
        ##flightModeService = rospy.ServiceProxy(
            ##'/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
        ##isModeChanged = flightModeService(
            ##custom_mode='GUIDED')  # return true or false
    
    ##except rospy.ServiceException as e:
        ##print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)


def waypoint():
    package = 'trajectory_a'
    launch_file = 'waypoint1.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

def menu():
    print("MULTIDRONE-COMMANDER")
    print("Press")
    print("1: to set mode to ARM the drone")
    print("2: to set mode to TAKEOFF")
    print("3: upload WAYPOINT TRAJECTORY")
    print("4: start WAYPOINT TRAJECTORY")
    print("5: to set mode to DISARM the drone") 
    print("6: to set mode to LAND")
    print("7: to set mode to HOLD")
    print("8: to set OFF-BOARD MODE")
    


def myLoop():
    x = '1'
    while ((not rospy.is_shutdown()) and (x in ['1', '2', '3', '4', '5', '6'])):
        menu()
        x = (input("Enter your input: "))
        if (x == '1'):
            setArm()
        elif (x == '2'):
            setTakeoffMode()
        elif (x == '3'):
            waypoint()
        elif (x == '4'):
            startWaypoint()
        elif (x == '5'):
            setDisarm()
        elif (x == '6'):
            setLandMode()               
        elif (x == '7'):
            setHoldMode
        elif (x == '8'):
            setOffboardMode                       
        else:
            print("Exit")
  

if __name__ == '__main__':
    rospy.init_node('dronemap_node', anonymous=False)
    rate = rospy.Rate(20)
    global sp_pub

    myLoop()
