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

Eigen::Vector3d cable_pos;
geometry_msgs::Twist tcp_vel;
ros::Publisher arm_pub;

void ArmShutdownHandler(int sig) {
    tcp_vel.linear.x = 0;
    tcp_vel.linear.y = 0;
    arm_pub.publish(tcp_vel);
    ros::shutdown();
}

void pca_cb(const geometry_msgs::PoseStamped& p) {
    cable_pos(0) = p.pose.position.x;
    cable_pos(1) = p.pose.position.y;
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

    while (ros::ok()) {
        // Get cable pose from GelSight
        double y = cable_pos(1);
        double theta = 0.0;

        // Calculate model state (y, theta, alpha)
        double alpha = 0.0;  // Use x, y and y_global
        Eigen::Vector3d x(y, theta, alpha);

        // Calculate phi from K
        std::cout << "x: " << x << std::endl;

        Eigen::Vector3d K(-200, 0, 0);
        double phi = -K.dot(x);
        std::cout << "phi: " << phi << std::endl;

        // Calculate velocity command from phi
        double target_dir = phi + alpha;
        target_dir =
            fmax(-3.14159265 / 3.0, fmin(target_dir, 3.14159265 / 3.0));

        // std::cout << target_dir << std::endl;

        // Publish velocity to arm
        tcp_vel.linear.x = -VNORM * cos(target_dir);
        tcp_vel.linear.y = VNORM * sin(target_dir);

        std::cout << "x: " << tcp_vel.linear.x << std::endl;
        std::cout << "y: " << tcp_vel.linear.y << std::endl;

        arm_pub.publish(tcp_vel);

        hz.sleep();
        ros::spinOnce();
    }

    return 0;
}

