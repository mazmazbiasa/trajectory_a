// Multi-UAVs (3 Drone) dengan WayPoint

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#define FLIGHT_ALTITUDE 4

mavros_msgs::State current_state_uav0;
mavros_msgs::State current_state_uav1;
mavros_msgs::State current_state_uav2;

geometry_msgs::PoseStamped pose_uav0;
geometry_msgs::PoseStamped pose_uav1;
geometry_msgs::PoseStamped pose_uav2;

// Subscriber Define
ros::Subscriber local_pos_sub_uav0;
ros::Subscriber local_pos_sub_uav1;
ros::Subscriber local_pos_sub_uav2;
ros::Subscriber state_sub_uav0;
ros::Subscriber pos_sub_uav0;
ros::Subscriber state_sub_uav1;
ros::Subscriber state_sub_uav2;
ros::Subscriber init_local_pos_sub_uav0;
ros::Subscriber init_local_pos_sub_uav1;
ros::Subscriber init_local_pos_sub_uav2;
ros::Subscriber local_vel_sub_uav0;
ros::Subscriber local_vel_sub_uav1;
ros::Subscriber local_vel_sub_uav2;

// Publisher Define
ros::Publisher local_pos_pub_uav0;
ros::Publisher local_pos_pub_uav1;
ros::Publisher local_pos_pub_uav2;
ros::Publisher local_vel_pub_uav0;
ros::Publisher local_vel_pub_uav1;
ros::Publisher local_vel_pub_uav2;
ros::ServiceClient arming_client_uav0;
ros::ServiceClient arming_client_uav1;
ros::ServiceClient arming_client_uav2;
ros::ServiceClient set_mode_client_uav0;
ros::ServiceClient set_mode_client_uav1;
ros::ServiceClient set_mode_client_uav2;
ros::ServiceClient land_client_uav0;
ros::ServiceClient land_client_uav1;
ros::ServiceClient land_client_uav2;


void state_cb_uav0(const mavros_msgs::State::ConstPtr& msg1){
    current_state_uav0 = *msg1;
}

void state_cb_uav1(const mavros_msgs::State::ConstPtr& msg2){
    current_state_uav1 = *msg2;
}

void state_cb_uav2(const mavros_msgs::State::ConstPtr& msg3){
    current_state_uav2 = *msg3;
}

void follow(const geometry_msgs::PoseStamped::ConstPtr& fmsg){
pose_uav0 = *fmsg;
pose_uav0.pose.position.x = pose_uav0.pose.position.x+1.0;
local_pos_pub_uav0.publish(pose_uav0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint1_node");
    ros::NodeHandle nh_uav0;
    ros::NodeHandle nh_uav1;
    ros::NodeHandle nh_uav2;

    // subs and pubs for uavs

    // ros::Publisher pub = node_handle.advertise<message_type>(topic_name, queue_size);
    
    state_sub_uav0 = nh_uav0.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb_uav0);
    local_pos_pub_uav0 = nh_uav0.advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    arming_client_uav0 = nh_uav0.serviceClient<mavros_msgs::CommandBool>
            ("uav0/mavros/cmd/arming");
    land_client_uav0 = nh_uav0.serviceClient<mavros_msgs::CommandTOL>
            ("uav0/mavros/cmd/land");
    set_mode_client_uav0 = nh_uav0.serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");
    // pos_sub_uav0 = nh_uav0.subscribe<geometry_msgs::PoseStamped>
        // ("uav0/mavros/local_position/pose", 10, follow);

    state_sub_uav1 = nh_uav1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb_uav1);
    local_pos_pub_uav1 = nh_uav1.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 10);
    arming_client_uav1 = nh_uav1.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    land_client_uav1 = nh_uav1.serviceClient<mavros_msgs::CommandTOL>
            ("uav1/mavros/cmd/land");
    set_mode_client_uav1 = nh_uav1.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    // pos_sub_uav0 = nh_uav0.subscribe<geometry_msgs::PoseStamped>
    //        ("uav0/mavros/local_position/pose", 10, follow);

    
    state_sub_uav2 = nh_uav2.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, state_cb_uav2);
    local_pos_pub_uav2 = nh_uav2.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 10);
    arming_client_uav2 = nh_uav2.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");
    land_client_uav2 = nh_uav2.serviceClient<mavros_msgs::CommandTOL>
            ("uav2/mavros/cmd/land");
    set_mode_client_uav2 = nh_uav2.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");
    // pos_sub_uav0 = nh_uav0.subscribe<geometry_msgs::PoseStam
    //    ("uav0/mavros/local_position/pose", 10, follow);
    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0);

    // wait for FCU connection
    while(ros::ok() && current_state_uav0.connected && current_state_uav1.connected && current_state_uav2.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCU...");
    }

    geometry_msgs::PoseStamped pose_uav0;
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = 4;
    geometry_msgs::PoseStamped pose_uav1;
    pose_uav1.pose.position.x = 0;
    pose_uav1.pose.position.y = 0;
    pose_uav1.pose.position.z = pose_uav0.pose.position.z;
    geometry_msgs::PoseStamped pose_uav2;
    pose_uav2.pose.position.x = 0;
    pose_uav2.pose.position.y = 0;
    pose_uav2.pose.position.z = pose_uav1.pose.position.z;



    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){

        local_pos_pub_uav2.publish(pose_uav2);
        local_pos_pub_uav1.publish(pose_uav1);
        local_pos_pub_uav0.publish(pose_uav0);
        ros::spinOnce();
        rate.sleep();
    }

//uav0 mode and commands
    mavros_msgs::SetMode offb_set_mode_uav0;
    offb_set_mode_uav0.request.custom_mode = "OFFBOARD";
    offb_set_mode_uav0.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd_uav0;
    arm_cmd_uav0.request.value = true;

    mavros_msgs::CommandTOL land_cmd_uav0;
    land_cmd_uav0.request.yaw = 0;
    land_cmd_uav0.request.latitude = 0;
    land_cmd_uav0.request.longitude = 0;
    land_cmd_uav0.request.altitude = 0;

//uav1 mode and commands
    mavros_msgs::SetMode offb_set_mode_uav1;
    offb_set_mode_uav1.request.custom_mode = "OFFBOARD"; //GUIDED
    offb_set_mode_uav1.request.custom_mode = "AUTO.TAKEOFF";
    mavros_msgs::CommandBool arm_cmd_uav1;
    arm_cmd_uav1.request.value = true;

    mavros_msgs::CommandTOL land_cmd_uav1;
    land_cmd_uav1.request.yaw = 0;
    land_cmd_uav1.request.latitude = 0;
    land_cmd_uav1.request.longitude = 0;
    land_cmd_uav1.request.altitude = 0;

//uav2 mode and commands

    mavros_msgs::SetMode offb_set_mode_uav2;
    offb_set_mode_uav2.request.custom_mode = "OFFBOARD"; //GUIDED
    offb_set_mode_uav2.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd_uav2;
    arm_cmd_uav2.request.value = true;

    mavros_msgs::CommandTOL land_cmd_uav2;
    land_cmd_uav2.request.yaw = 0;
    land_cmd_uav2.request.latitude = 0;
    land_cmd_uav2.request.longitude = 0;
    land_cmd_uav2.request.altitude = 0;


    ros::Time last_request = ros::Time::now();

   //WP-1
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("WAYPOINT INPUT DETECTED");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("MULTI UAV READY TO GO");

    //WP-2
    pose_uav0.pose.position.x = 6;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (6,0)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (6,0)");

    //WP-3
    pose_uav0.pose.position.x = 12;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (12,0)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){
      
      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (12,0)");
    
    //WP-4
    pose_uav0.pose.position.x = 18;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (18,0)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (18,0)");


    //WP-5
    pose_uav0.pose.position.x = 24;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (24,0)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (24,0)");


    //WP-6
    pose_uav0.pose.position.x = 24;
    pose_uav0.pose.position.y = 7;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (24,7)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (24,7)");

    //WP-7
    pose_uav0.pose.position.x = 18;
    pose_uav0.pose.position.y = 7;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (18,7)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (18,7)");

    //WP-8
    pose_uav0.pose.position.x = 12;
    pose_uav0.pose.position.y = 7;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (12,7)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (12,7)");

    //WP-9
    pose_uav0.pose.position.x = 6;
    pose_uav0.pose.position.y = 7;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (6,7)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (6,7)");

    //WP-10
    pose_uav0.pose.position.x = 6;
    pose_uav0.pose.position.y = 14;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (6,14)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (6,14)");

    //WP-11
    pose_uav0.pose.position.x = 12;
    pose_uav0.pose.position.y = 14;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (12,14)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (12,14)");

    //WP-12
    pose_uav0.pose.position.x = 18;
    pose_uav0.pose.position.y = 14;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (18,14)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (18,14)");
    
    //WP-13
    pose_uav0.pose.position.x = 24;
    pose_uav0.pose.position.y = 14;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (24,14)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (24,14)");
    
    //WP-14
    pose_uav0.pose.position.x = 24;
    pose_uav0.pose.position.y = 21;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (24,21)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (24,21)");

    //WP-15
    pose_uav0.pose.position.x = 18;
    pose_uav0.pose.position.y = 21;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (18,21)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (18,21)");

    //WP-16
    pose_uav0.pose.position.x = 12;
    pose_uav0.pose.position.y = 21;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (12,21)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (12,21)");

    //WP-17
    pose_uav0.pose.position.x = 6;
    pose_uav0.pose.position.y = 21;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO (6,21)");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT (6,21)");

    //WP-18
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("GO TO HOME");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub_uav0.publish(pose_uav0);
      local_pos_pub_uav1.publish(pose_uav1);
      local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ALREADY AT HOME");


    
    

    //pose_uav0.pose.position.x = 3;
    //pose_uav0.pose.position.y = 3;
    //pose_uav0.pose.position.z = FLIGHT_ALTITUDE;
    
    //pose_uav1.pose.position.x = pose_uav0.pose.position.x;
    //pose_uav1.pose.position.y = pose_uav0.pose.position.y;
    //pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    //pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    //pose_uav2.pose.position.y = pose_uav1.pose.position.y;
    //pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    //ROS_INFO("LAND POSITION");
    //send setpoints for 10 seconds
    //for(int i = 0; ros::ok() && i < 10*930; ++i){

      //local_pos_pub_uav0.publish(pose_uav0);
      //local_pos_pub_uav1.publish(pose_uav1);
      //local_pos_pub_uav2.publish(pose_uav2);S

      //ros::spin();
      //rate.sleep();
    //}

}