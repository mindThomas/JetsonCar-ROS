/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifndef LSPC_CALLBACKS_H
#define LSPC_CALLBACKS_H

#include <vector>
#include <memory>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void LSPC_Callback_Sensors(ros::Publisher& pubEncoders, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom, const std::vector<uint8_t>& payload);
void LSPC_Callback_CPUload(ros::Publisher& pubLoad, const std::vector<uint8_t>& payload);
void LSPC_Callback_ArrayDump(std::shared_ptr<std::ofstream> log_file, const std::vector<uint8_t>& payload);
void LSPC_Callback_Debug(ros::Publisher& pubDebug, const std::vector<uint8_t>& payload);

#endif // LSPC_CALLBACKS_H
