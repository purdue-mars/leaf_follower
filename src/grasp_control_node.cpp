#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>

#include "geometry_msgs/PoseStamped.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"

#define MAX_GRASP_POS 2.2
#define STABLE_CONTACT_AREA 15
#define V_MAX 0.1
#define Kp 0.01
#define Kd 0.00
#define JOINT_NAME "gripper_joint"
#define LIM_HI 2.5
#define LIM_LOW 0.0

float contact_area;
float joint_state;  // rad
sensor_msgs::JointState gripper_state;
ros::Publisher gripper_pub;

void GripperShutdownHandler(int sig) {
    gripper_state.velocity[0] = 0;
    gripper_pub.publish(gripper_state);
    ros::shutdown();
}

void contact_cb(const std_msgs::Float32& c) { contact_area = c.data; }

void joint_state_cb(const sensor_msgs::JointState& s) {
    if (s.name[0] == JOINT_NAME) {
        joint_state = s.position[0];
    }
}

void set_joint_state(float grasp_vel, sensor_msgs::JointState& out) {
    if (joint_state >= LIM_HI || joint_state <= LIM_LOW) {
        // pos if near HI, neg if near low
        float jnt_state_sign = joint_state - (LIM_HI - LIM_LOW) / 2.0;

        // prevent velocities that go against joint limit
        if (jnt_state_sign * grasp_vel > 0) {
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

    ros::Rate hz(20);
    gripper_state.name.push_back(JOINT_NAME);
    gripper_state.position.push_back(0);
    gripper_state.velocity.push_back(0);

    float prev_err = 0.0;

    while (ros::ok()) {
        float err = STABLE_CONTACT_AREA - contact_area;
        float ctrl = Kp * err + Kd * (err - prev_err);
        prev_err = err;

        ctrl = fmax(-V_MAX, fmin(ctrl, V_MAX));
        set_joint_state(ctrl, gripper_state);
        gripper_pub.publish(gripper_state);
        hz.sleep();
        ros::spinOnce();
    }

    return 0;
}

