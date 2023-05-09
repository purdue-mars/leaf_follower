#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define VNORM 0.02

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

void tf_twist_A_to_B(Eigen::Affine3d& T, Eigen::Vector3d& v_a,
                     Eigen::Vector3d& w_a, Eigen::Vector3d& v_b,
                     Eigen::Vector3d& w_b) {
    Eigen::Matrix<double, 6, 6> adj_map;
    adj_map.topLeftCorner<3, 3>() = T.rotation();
    adj_map.bottomRightCorner<3, 3>() = T.rotation();
    Eigen::Vector3d p = T.linear().col(0);
    Eigen::Matrix3d skew_sym;
    skew_sym << 0, -p(2), p(1), p(2), 0, -p(0), -p(1), p(0), 0;
    adj_map.bottomLeftCorner<3, 3>() = skew_sym * T.rotation();
    Eigen::Matrix<double, 6, 1> twist_a, twist_b;
    twist_a.head<3>() = v_a;

    twist_b = adj_map * twist_a;
    v_b = twist_b.head<3>();
    w_b = twist_b.tail<3>();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "follower_node");

    ros::NodeHandle nh;
    signal(SIGINT, ArmShutdownHandler);
    ros::Subscriber diff_pose_sub =
        nh.subscribe("/gelsight/diff_pose", 1000, pca_cb);
    arm_pub =
        nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 2);

    ros::Rate hz(50);
    tf::TransformListener tf_listener;

    Eigen::Affine3d T_base_tcp;

    bool initialized = false;
    double Kp_th = 600;
    double Kd_th = 10;
    double Kp_y = 200;
    double Kd_y = 10;
    double prev_theta = 0;
    double prev_y = 0;

    while (ros::ok()) {
        try {
            tf::StampedTransform transform;
            tf_listener.lookupTransform("tool0", "base_link", ros::Time(0),
                                        transform);
            tf::transformTFToEigen(transform, T_base_tcp);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
        }

        // Get cable pose from GelSight
        double y = cable_pos(0);
        auto angles = cable_angle.toRotationMatrix().eulerAngles(0, 1, 2);
        double theta = angles[2];

        theta = -PI / 2 - theta;

        std::cout << theta << std::endl;
        double phi = Kp_y * y + Kd_y * (y - prev_y);
        theta = Kp_th * theta + Kd_th * (theta - prev_theta);
        prev_theta = theta;
        prev_y = y;

        // Calculate velocity command from phi
        phi = fmax(-PI / 3.0, fmin(phi, PI / 3.0));
        theta = fmax(-5, fmin(theta, 5));

        // std::cout << target_dir << std::endl;

        // Publish velocity to arm
        Eigen::Vector3d v, w;
        v = Eigen::Vector3d::Zero();
        w = Eigen::Vector3d::Zero();

        v(1) = VNORM * cos(phi);
        v(2) = VNORM * sin(phi);
        w(0) = VNORM * theta;

        // transforms v,w from tool frame to base_frame
        tf_twist_A_to_B(T_base_tcp, v, w, v, w);

        tcp_vel.linear.x = v(0);
        tcp_vel.linear.y = v(1);
        tcp_vel.linear.z = v(2);
        tcp_vel.angular.x = w(0);
        tcp_vel.angular.y = w(1);
        tcp_vel.angular.z = w(2);

        arm_pub.publish(tcp_vel);

        hz.sleep();
        ros::spinOnce();
    }

    return 0;
}

