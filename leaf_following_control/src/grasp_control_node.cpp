#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>

#include "geometry_msgs/PoseStamped.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h" // Include for Bool message

// tune for object

#define STABLE_CONTACT_AREA 150
#define V_MAX 0.1
#define Kp 0.001
#define Kd 0.0001
#define JOINT_NAME "gripper_joint"
#define LIM_HI 1.88 // closed
#define LIM_LOW 0.0 // open

float contact_area;
float joint_state;  // rad
sensor_msgs::JointState gripper_state;
ros::Publisher gripper_pub;
bool start_processing = false; // Global flag to control processing
bool joint_state_RT_flag = false;
bool contact_area_RT_flag = false;

void start_cb(const std_msgs::Bool& start_signal) {
    start_processing = start_signal.data; // Set flag based on the received signal
}


void GripperShutdownHandler(int sig) {
    gripper_state.velocity[0] = 0;
    gripper_pub.publish(gripper_state);
    ros::shutdown();
}

void contact_cb(const std_msgs::Float32& c) { 
   contact_area = c.data; 
   contact_area_RT_flag = true;
}

void joint_state_cb(const sensor_msgs::JointState& s) {
    if (s.name[0] == JOINT_NAME) {
        joint_state = s.position[0];
    }
    joint_state_RT_flag = true;
}

void set_joint_state(float grasp_vel, sensor_msgs::JointState& out) {
    std::cout << "jnt state: " << joint_state << std::endl;
    if (joint_state >= LIM_HI) {
        if (grasp_vel > 0.0) {
            grasp_vel = 0.0;
        }
    }
    else if(joint_state <= LIM_LOW) {
        if (grasp_vel < 0.0) {
            grasp_vel = 0.0;
        }
    }
    out.velocity[0] = grasp_vel;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_control_node");

    ros::NodeHandle nh;
    signal(SIGINT, GripperShutdownHandler);
    ros::Subscriber diff_contact_sub =
        nh.subscribe("/gelsight/diff_contact", 1000, contact_cb);
    ros::Subscriber joint_state_sub =
        nh.subscribe("/joint_states", 1000, joint_state_cb);
    gripper_pub =
        nh.advertise<sensor_msgs::JointState>("/desired_joint_states", 10);
    // Subscribe to the start signal topic
    ros::Subscriber start_sub = nh.subscribe("/gripper_start_signal", 10, start_cb);
    
    ros::Rate hz(7);
    gripper_state.name.push_back(JOINT_NAME);
    gripper_state.position.push_back(0);
    gripper_state.velocity.push_back(0);

    float prev_err = 0.0;
    float ctrl = 0.0;
    float err = 0.0;

    while (ros::ok()) {
        joint_state_RT_flag = false;
        contact_area_RT_flag = false;
    	ros::spinOnce(); // Still need to call spinOnce to process callbacks
	if (start_processing) { // Check if processing should start            
	    err = STABLE_CONTACT_AREA - contact_area;
            ctrl = Kp * err + Kd * (err - prev_err);
            prev_err = err;
            ctrl = fmax(-V_MAX, fmin(ctrl, V_MAX));
	    std::cout << "grasping!" << std::endl;
        } 
	else{
	    // open gripper
	    ctrl = -V_MAX;
	}
	if(!joint_state_RT_flag) { // check gripper joint state is recieved
	    ROS_ERROR("JOINT STATE MESSAGE MISSING!");
	    ctrl = 0;
	}
	else if(!contact_area_RT_flag) { // check gelsight contact area is recieved
	    ROS_ERROR("GELSIGHT CONTACT AREA MESSAGE MISSING!");
	    ctrl = 0;
	}
	std::cout << "ctrl: " << ctrl << std::endl;
        set_joint_state(ctrl, gripper_state); 
	std::cout << gripper_state.velocity[0] << std::endl;
        gripper_pub.publish(gripper_state);
    	hz.sleep();
    }

    return 0;
}

