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

#include "jetsoncar_driver/odometry.h"

#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDrive.h"

/* Global variables */
double min_steering = 0.003; // rad
double max_steering = 0.5; // rad
double min_omega = 0.01; // rad/s
double wheel_radius = 0.155f / 2.f; // m
double front_to_rear_wheel_distance = 0.42; // m
double left_to_right_wheel_distance = 0.285; // m

void PublishOdometry(double v, double rho, double dt, ros::Time time, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom)
{
    static double x_prev = 0;
    static double y_prev = 0;
    static double theta_prev = 0;

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
    //std::cout << "v = " << v << std::endl;

    if (std::abs(rho) > min_steering) {
        double beta = std::atan((front_to_rear_wheel_distance / 2) / front_to_rear_wheel_distance * std::tan(rho));
        omega = v * std::cos(beta) * std::tan(rho) / front_to_rear_wheel_distance;

        /*std::cout << "rho = " << rho << std::endl;
        std::cout << "beta = " << beta << std::endl;
        std::cout << "omega = " << omega << std::endl;*/

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

    /*
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
    */

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
