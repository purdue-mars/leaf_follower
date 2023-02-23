#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>

#include "geometry_msgs/PoseStamped.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "tf/LinearMath/Transform.h"

#define VNORM 0.005

#define PI 3.14159265

Eigen::Vector3d cable_pos;
Eigen::Quaterniond cable_angle;
geometry_msgs::Twist tcp_vel;
ros::Publisher arm_pub;

void ArmShutdownHandler(int sig) {
    tcp_vel.linear.x = 0;
    tcp_vel.linear.y = 0;
    tcp_vel.angular.z = 0;
    arm_pub.publish(tcp_vel);
    ros::shutdown();
}

void pca_cb(const geometry_msgs::PoseStamped& p) {
    cable_pos(0) = p.pose.position.x;
    cable_pos(1) = p.pose.position.y;

    cable_angle.w() = p.pose.orientation.w;
    cable_angle.x() = p.pose.orientation.x;
    cable_angle.y() = p.pose.orientation.y;
    cable_angle.z() = p.pose.orientation.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "follower_node");

    ros::NodeHandle nh;
    signal(SIGINT, ArmShutdownHandler);
    ros::Subscriber diff_pose_sub =
        nh.subscribe("/gelsight/diff_pose", 1000, pca_cb);
    arm_pub =
        nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 2);

    ros::Rate hz(20);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    Eigen::Affine3d T_O_tcp;

    bool initialized = false;
    double K_p_th = 600;
    double K_p_y = 200;

    while (ros::ok()) {
        // Get cable pose from GelSight
        double y = cable_pos(0);
        auto angles = cable_angle.toRotationMatrix().eulerAngles(0, 1, 2);
        double theta = angles[2];

        theta = -PI / 2 - theta;

        std::cout << theta << std::endl;
        double phi = K_p_y * y;
        theta = K_p_th * theta;

        // Calculate velocity command from phi
        phi = fmax(-PI / 3.0, fmin(phi, PI / 3.0));
        theta = fmax(-5, fmin(theta, 5));

        // std::cout << target_dir << std::endl;

        // Publish velocity to arm
        tcp_vel.linear.x = -VNORM * cos(phi);
        tcp_vel.linear.y = VNORM * sin(phi);
        tcp_vel.angular.z = VNORM * theta;

        std::cout << "x: " << tcp_vel.linear.x << std::endl;
        std::cout << "y: " << tcp_vel.linear.y << std::endl;
        std::cout << "z_th: " << tcp_vel.angular.z << std::endl;

        arm_pub.publish(tcp_vel);

        hz.sleep();
        ros::spinOnce();
    }

    return 0;
}

