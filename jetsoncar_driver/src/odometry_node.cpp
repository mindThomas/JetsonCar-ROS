/* Copyright (C) 2019-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <ros/ros.h>

/* Include standard libraries */
#include <string>
#include <boost/bind.hpp>
#include <cmath>

/* Include ROS libraries */
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>

/* Include message types */
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDrive.h"

/* Include generated Services */

/* Include generated Message Types */
#include <jetsoncar_msgs/Encoders.h>

/* Include generated Dynamic Reconfigure parameters */

// For CLion to update/capture the changes made to generated services, message types and parameters, open the "build" folder and run "make"


/* This node Subscribes to the /encoders message and integrates that using the Bicycle model into
 * odometry messages (/odom) and odometry transformations (odom frame) */

/* Global variables */
std::string tfPrefix;
double min_steering = 0.003;
double max_steering = 1.0;
double min_omega = 0.01;
double wheel_radius = 1;
double front_to_rear_wheel_distance = 1;
double left_to_right_wheel_distance = 1;

ros::Time prev_odometry_time(0);
double prev_front_angle = 0;
double prev_rear_angle = 0;
double prev_steering_angle = 0;
double x_prev = 0;
double y_prev = 0;
double theta_prev = 0;

void PublishOdometry(double v, double rho, double dt, ros::Time time, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom)
{
    // Use the Bicycle Kinematic model with the origin (position) defined in the center of mass
    // which is assumed to be the midpoint between front and rear wheels (base_link frame)
    // https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-the-kinematic-bicycle-model-Bi8yE
    // https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
    // dx = v * cos(theta + beta)
    // dy = v * sin(theta + beta)
    // dtheta = v * cos(beta) * tan(rho) / L
    // beta = atan(l_r/L * tan(rho))
    // Where
    //   L   : distance between front to rear wheels
    //   l_r : distance between rear wheels to center of mass (midpoint)
    //   rho : steering angle
    // l_r is assumed to be L/2

    // To discretize this we assume the velocity and steering angle is constant during the discretization interval, dt
    // In that case the Coordinated Turn Model can be used
    // See Chapter 5.2.2 of http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.142.6137&rep=rep1&type=pdf
    // x[k+1] = x[k] + 2*v[k]/omega[k] * sin(dt/2*omega[k]) * cos(theta[k] + beta[k] + dt/2*omega[k])
    // y[k+1] = x[k] + 2*v[k]/omega[k] * sin(dt/2*omega[k]) * sin(theta[k] + beta[k] + dt/2*omega[k])
    // theta[k+1] = theta[k] + dt*omega[k]
    // Where
    //    omega[k] = v[k] * cos(beta[k]) * tan(rho[k]) / L
    // A different form can be found in equation 5.9 on page 101 of Probabilistic Robotics by Sebastian Thrun

    double x = x_prev;
    double y = y_prev;
    double theta = theta_prev;
    double omega = 0;
    std::cout << "v = " << v << std::endl;

    if (std::abs(rho) > min_steering) {
        double beta = std::atan((front_to_rear_wheel_distance / 2) / front_to_rear_wheel_distance * std::tan(rho));
        omega = v * std::cos(beta) * std::tan(rho) / front_to_rear_wheel_distance;

        std::cout << "rho = " << rho << std::endl;
        std::cout << "beta = " << beta << std::endl;
        std::cout << "omega = " << omega << std::endl;

        if (std::abs(omega) > min_omega) {
            // Compute predicted odometry position
            x += 2 * v / omega * std::sin(dt / 2 * omega) * std::cos(theta_prev + beta + dt / 2 * omega);
            y += 2 * v / omega * std::sin(dt / 2 * omega) * std::sin(theta_prev + beta + dt / 2 * omega);
            theta += dt * omega;
        } else {
            x += dt * cos(theta) * v;
            y += dt * sin(theta) * v;
        }
    } else {
        x += dt * cos(theta) * v;
        y += dt * sin(theta) * v;
    }

    double dx_est, dy_est;
    dx_est = (x - x_prev) / dt;
    dy_est = (y - y_prev) / dt;

    // Construct odometry transform and publish
    tf::Transform odom_transform;
    odom_transform.setOrigin(tf::Vector3(x, y, 0));
    odom_transform.setRotation(tf::createQuaternionFromYaw(theta));
    tfBroadcaster.sendTransform(tf::StampedTransform(odom_transform, time, "odom", "footprint"));

    std::cout.precision(3);
    std::cout.setf(std::ios::fixed,std::ios::floatfield);
    std::cout << "At time " << time.toSec() << std::endl;
    double yaw, pitch, roll;
    odom_transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = odom_transform.getRotation();
    tf::Vector3 tv = odom_transform.getOrigin();
    std::cout << "- Translation: [" << tv.getX() << ", " << tv.getY() << ", " << tv.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
              << q.getZ() << ", " << q.getW() << "]" << std::endl
              << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
              << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;

    /* Send odometry message */
    //   This represents an estimate of a position and velocity in free space.
    //   The pose in this message should be specified in the coordinate frame given by header.frame_id
    //   The twist in this message should be specified in the coordinate frame given by the child_frame_id
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "footprint";
    odom_msg.pose.pose.position.x = x; // inertial frame position
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.w = odom_transform.getRotation().w();
    odom_msg.pose.pose.orientation.x = odom_transform.getRotation().x();
    odom_msg.pose.pose.orientation.y = odom_transform.getRotation().y();
    odom_msg.pose.pose.orientation.z = odom_transform.getRotation().z();
    odom_msg.twist.twist.linear.x = v; // body frame velocity
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = omega;

    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[14] = 1000000000000.0;
    odom_msg.pose.covariance[21] = 1000000000000.0;
    odom_msg.pose.covariance[28] = 1000000000000.0;

    odom_msg.pose.covariance[35] = 0.01; // std::abs(angular_vel.z) < 0.0001
    //odom_msg.pose.covariance[35] = 100.0;

    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[7] = 0.001;
    odom_msg.twist.covariance[14] = 0.001;
    odom_msg.twist.covariance[21] = 1000000000000.0;
    odom_msg.twist.covariance[28] = 1000000000000.0;
    odom_msg.twist.covariance[35] = 0.01; // std::abs(angular_vel.z) < 0.0001
    //odom_msg.twist.covariance[35] = 100.0;

    pubOdom.publish(odom_msg);

    // Store updated states for next iteration
    x_prev = x;
    y_prev = y;
    theta_prev = theta;
}



void ROS_Callback_encoders(const jetsoncar_msgs::Encoders::ConstPtr& msg, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom)
{
    auto time_diff = msg->receive_time - prev_odometry_time;
    double dt = time_diff.toSec();

    if (dt > 0 && prev_odometry_time.toSec() != 0) {
        // Since we want to use and integrate the encoder measurements, we will compute the instantaneous velocity by
        // differentiating the encoder readings
        double front_velocity = (msg->front_angle - prev_front_angle) / dt;
        double rear_velocity = (msg->front_angle - prev_front_angle) / dt;
        double wheel_velocity = (front_velocity + rear_velocity) / 2.0;

        // Compute necessary variables
        double v = wheel_radius * wheel_velocity; // translational velocity
        double rho = msg->steering_angle;

        PublishOdometry(v, rho, dt, msg->receive_time, tfBroadcaster, pubOdom);
    }

    prev_odometry_time = msg->receive_time;
    prev_front_angle = msg->front_angle;
    prev_rear_angle = msg->rear_angle;
    prev_steering_angle = msg->steering_angle;
}

void ROS_Callback_cmd_ackermann(const ackermann_msgs::AckermannDrive::ConstPtr& msg, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom)
{
    ros::Time current_time = ros::Time::now();

    // Convert cmd_vel command into actuation commends sent to the Gazebo simulated wheel controllers
    double speed = msg->speed;
    double steering_angle = msg->steering_angle; // misuse of using this to capture the steering angle

    auto time_diff = current_time - prev_odometry_time;
    double dt = time_diff.toSec();

    if (dt > 0 && prev_odometry_time.toSec() != 0) {
        PublishOdometry(speed, steering_angle, dt, current_time, tfBroadcaster, pubOdom);
    }

    prev_odometry_time = current_time;
}

int main(int argc, char **argv) {
	std::string nodeName = "odometry_node";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n; // default/current namespace node handle
	ros::NodeHandle nParam("~"); // default/current namespace node handle for parameters

    tfPrefix = tf::getPrefixParam(n);
	tf::TransformBroadcaster tfBroadcaster;

	bool use_encoders = true;
    if (!nParam.getParam("use_encoders", use_encoders)) {
        ROS_WARN_STREAM("Use encoders flag not set (Parameter: use_encoders). Defaults to: " << use_encoders);
    }

    if (!nParam.getParam("wheel_radius", wheel_radius)) {
        ROS_WARN_STREAM("Wheel radius not set (Parameter: wheel_radius). Defaults to: " << wheel_radius);
    }
    if (!nParam.getParam("front_to_rear_wheel_distance", front_to_rear_wheel_distance)) {
        ROS_WARN_STREAM("Wheel distance not set (Parameter: front_to_rear_wheel_distance). Defaults to: " << front_to_rear_wheel_distance);
    }
    if (!nParam.getParam("left_to_right_wheel_distance", left_to_right_wheel_distance)) {
        ROS_WARN_STREAM("Wheel distance not set (Parameter: front_to_rear_wheel_distance). Defaults to: " << left_to_right_wheel_distance);
    }
    if (!nParam.getParam("min_steering", min_steering)) {
        ROS_WARN_STREAM("Minimum steering angle not set (Parameter: min_steering). Defaults to: " << min_steering);
    }
    if (!nParam.getParam("max_steering", max_steering)) {
        ROS_WARN_STREAM("Maximum steering angle not set (Parameter: max_steering). Defaults to: " << max_steering);
    }
    if (!nParam.getParam("min_omega", min_omega)) {
        ROS_WARN_STREAM("Minimum angular velocity set (Parameter: min_omega). Defaults to: " << min_omega);
    }

    /* Configure publishers */
    ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odom", 50);

    /* Configure subscribers */
    ros::Subscriber sub;
    if (use_encoders)
        sub = n.subscribe<jetsoncar_msgs::Encoders>("encoders", 50, boost::bind(&ROS_Callback_encoders, _1, boost::ref(tfBroadcaster), boost::ref(pub_odom)));
    else
        sub = n.subscribe<ackermann_msgs::AckermannDrive>("cmd_ackermann", 50, boost::bind(&ROS_Callback_cmd_ackermann, _1, boost::ref(tfBroadcaster), boost::ref(pub_odom)));

    ros::spin();
}