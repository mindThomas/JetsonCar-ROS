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

#ifndef ROS_CALLBACKS_H
#define ROS_CALLBACKS_H

#include <memory>
#include <mutex>

#include "LSPC.h"
#include "MessageTypes.h"

// For CLion to update/capture the changes made to generated services, message types and parameters, open the "build" folder and run "make"
/* Include generated Services */
#include <jetsoncar_srvs/Reboot.h>
#include <jetsoncar_srvs/EnterBootloader.h>
#include <jetsoncar_srvs/SetPID.h>

/* Include generated Message Types */
#include <jetsoncar_msgs/Encoders.h>
#include <jetsoncar_msgs/Setpoint.h>

/* Include generated Dynamic Reconfigure parameters */
#include <dynamic_reconfigure/server.h>
#include <jetsoncar_driver/ParametersConfig.h>

// Global variables
extern bool MATLAB_log;
extern std::timed_mutex reconfigureMutex;
extern std::shared_ptr<dynamic_reconfigure::Server<jetsoncar_driver::ParametersConfig>> reconfigureServer;
extern jetsoncar_driver::ParametersConfig reconfigureConfig;

void ROS_Callback_Setpoint(const jetsoncar_msgs::Setpoint::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_Reboot(jetsoncar_srvs::Reboot::Request &req, jetsoncar_srvs::Reboot::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_EnterBootloader(jetsoncar_srvs::EnterBootloader::Request &req, jetsoncar_srvs::EnterBootloader::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
bool ROS_Service_SetPID(jetsoncar_srvs::SetPID::Request &req, jetsoncar_srvs::SetPID::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);
void ROS_ReconfigureCallback(jetsoncar_driver::ParametersConfig &config, uint32_t level, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj);

#endif // ROS_CALLBACKS_H
