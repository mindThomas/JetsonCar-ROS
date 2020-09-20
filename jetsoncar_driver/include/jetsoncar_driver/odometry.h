/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/* Global variables */
extern double min_steering;
extern double max_steering;
extern double min_omega;
extern double wheel_radius;
extern double front_to_rear_wheel_distance;
extern double left_to_right_wheel_distance;

void PublishOdometry(double v, double rho, double dt, ros::Time time, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom);

#endif // ODOMETRY_H
