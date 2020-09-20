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

#include <ros/ros.h>

#include "LSPC.h"
#include "MessageTypes.h"
#include <string>
#include <thread>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/format.hpp>
#include <future>
#include <boost/bind.hpp>
#include <fstream>      // for file output (std::ofstream)
#include <boost/filesystem.hpp>   // for directory creation (boost::filesystem)
#include <chrono>
#include <cstdint>
#include <tuple>
#include <cmath>

#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <locale>
#include <stdio.h>
#include <stdlib.h>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
/*#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>*/
#include <geometry_msgs/TransformStamped.h>

#include "jetsoncar_driver/jetsoncar_driver_node/ros_callbacks.h"
#include "jetsoncar_driver/jetsoncar_driver_node/lspc_callbacks.h"
#include "jetsoncar_driver/odometry.h"

#include <utils.hpp>

/* Include messages */
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

/* Include generated Dynamic Reconfigure parameters */
#include <dynamic_reconfigure/server.h>
#include <jetsoncar_driver/ParametersConfig.h>

void LSPC_ConnectionThread(boost::shared_ptr<ros::NodeHandle> n, std::string serial_port, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj, std::future<void> exitSignalObj)
{
    /* Configure transform publisher and listener */
    tf::TransformBroadcaster tf_broadcaster; // maybe wrap with unique_ptr ?
    std::shared_ptr<tf::TransformListener> tf_listener = std::make_shared<tf::TransformListener>();

    /* Configure ROS publishers (based on received LSPC messages) */
    ros::Publisher pub_odom = n->advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher pub_encoders = n->advertise<jetsoncar_msgs::Encoders>("encoders", 50);
    ros::Publisher pub_mcu_debug = n->advertise<std_msgs::String>("mcu_debug", 50);
    ros::Publisher pub_mcu_load = n->advertise<std_msgs::String>("mcu_load", 50);

    lspcMutex->lock();
    while (exitSignalObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
        *lspcObj = new lspc::Socket; // recreate a new LSPC object
        while (!(*lspcObj)->isOpen() && ros::ok()) {
            try {
                ROS_DEBUG("Trying to connect to JetsonCar");
                (*lspcObj)->open(serial_port);
            }
            catch (boost::system::system_error &e) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        if (!ros::ok()) break;

        ROS_DEBUG("Connected to JetsonCar");

        if (!boost::filesystem::is_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/jetsoncar_logs"))) {
            if (boost::filesystem::exists(boost::filesystem::path(std::string(getenv("HOME")) + "/jetsoncar_logs"))) {
                std::cout << "JetsonCar dump path (~/jetsoncar_logs) already exists but without write permissions" << std::endl;
            } else {
                if (!boost::filesystem::create_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/jetsoncar_logs")))
                    std::cout << "Could not create log folder (~/jetsoncar_logs)" << std::endl;
                else
                    std::cout << "Successfully created log folder (~/jetsoncar_logs)" << std::endl;
            }
        }

        std::shared_ptr<std::ofstream> sensorsFile = std::make_shared<std::ofstream>();

        lspcMutex->unlock();

        /* Register callbacks */
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::Sensors, boost::bind(&LSPC_Callback_Sensors, pub_encoders, boost::ref(tf_broadcaster), boost::ref(pub_odom), _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CPUload, boost::bind(&LSPC_Callback_CPUload, pub_mcu_load, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::Debug, boost::bind(&LSPC_Callback_Debug, pub_mcu_debug, _1));

        bool MATLAB_log_prev = false;
        while ((*lspcObj)->isOpen() && ros::ok()) {
            if (MATLAB_log != MATLAB_log_prev) {
                if (MATLAB_log) {
                    std::string dumpFilename = GetFormattedTimestampCurrent() + ".txt";
                    sensorsFile->open(std::string(getenv("HOME")) + "/jetsoncar_logs/" + dumpFilename, std::ofstream::trunc);
                    ROS_DEBUG_STREAM("Created mathdump file: ~/jetsoncar_logs/" << dumpFilename);
                } else {
                    sensorsFile->close();
                }
                MATLAB_log_prev = MATLAB_log;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!ros::ok())
            break;

        ROS_WARN("Connection lost to JetsonCar");
        lspcMutex->lock();
        delete(*lspcObj); // call destructor to disconnect completely and clean up used ressources

        sensorsFile->close();
    }
}

int main(int argc, char **argv) {
	std::string nodeName = "jersoncar_driver";
	ros::init(argc, argv, nodeName.c_str());
    boost::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);  // ros::NodeHandle n
	ros::NodeHandle nParam("~"); // default/current namespace node handle

	// Set Debug verbosity
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

	ROS_DEBUG_STREAM("Starting JetsonCar driver...");

	std::string serial_port = "/dev/jetsoncar";
	if (!nParam.getParam("serial_port", serial_port)) {
		ROS_WARN_STREAM("Serial port not set (Parameter: serial_port). Defaults to: /dev/jetsoncar");
	}

    if (!nParam.getParam("wheel_radius", wheel_radius)) {
        ROS_WARN_STREAM("Wheel radius not set (Parameter: wheel_radius). Defaults to: " << wheel_radius);
    }
    if (!nParam.getParam("front_to_rear_wheel_distance", front_to_rear_wheel_distance)) {
        ROS_WARN_STREAM("Wheel distance not set (Parameter: front_to_rear_wheel_distance). Defaults to: " << front_to_rear_wheel_distance);
    }
    if (!nParam.getParam("left_to_right_wheel_distance", left_to_right_wheel_distance)) {
        ROS_WARN_STREAM("Wheel distance not set (Parameter: left_to_right_wheel_distance). Defaults to: " << left_to_right_wheel_distance);
    }
    if (!nParam.getParam("min_steering", min_steering)) {
        ROS_WARN_STREAM("Minimum steering angle not set (Parameter: min_steering). Defaults to: " << min_steering);
    }
    if (!nParam.getParam("min_omega", min_omega)) {
        ROS_WARN_STREAM("Minimum angular velocity set (Parameter: min_omega). Defaults to: " << min_omega);
    }

    /* Create common LSPC object */
    std::shared_ptr<std::timed_mutex> lspcMutex = std::make_shared<std::timed_mutex>();
    std::shared_ptr<lspc::Socket *> lspcObj = std::make_shared<lspc::Socket *>(nullptr);  // read about shared_ptr here: https://www.acodersjourney.com/top-10-dumb-mistakes-avoid-c-11-smart-pointers/

    /* Configure subscribers */
    //std::shared_ptr<ros::Timer> reference_timeout_timer = std::make_shared<ros::Timer>(n->createTimer(ros::Duration(0.1), boost::bind(&ROS_Callback_Heartbeat, lspcMutex, lspcObj, reference_timeout_timer, _1))); // Configure 10 Hz heartbeat timer
    //ros::Subscriber sub_cmd_vel = n->subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ROS_Callback_cmd_vel, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_setpoint = n->subscribe<jetsoncar_msgs::Setpoint>("setpoint", 1, boost::bind(&ROS_Callback_Setpoint, _1, lspcMutex, lspcObj));

    /* Configure node services */
    ros::ServiceServer serv_reboot = n->advertiseService<jetsoncar_srvs::Reboot::Request, jetsoncar_srvs::Reboot::Response>("/jetsoncar/reboot", boost::bind(&ROS_Service_Reboot, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_enter_bootloader = n->advertiseService<jetsoncar_srvs::EnterBootloader::Request, jetsoncar_srvs::EnterBootloader::Response>("/jetsoncar/enter_bootloader", boost::bind(&ROS_Service_EnterBootloader, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_set_pi = n->advertiseService<jetsoncar_srvs::SetPID::Request, jetsoncar_srvs::SetPID::Response>("/jetsoncar/set_pid", boost::bind(&ROS_Service_SetPID, _1, _2, lspcMutex, lspcObj));

    /* Enable dynamic reconfigure */
    // Enable reconfigurable parameters - note that any parameters set on the node by roslaunch <param> tags will be seen by a dynamically reconfigurable node just as it would have been by a conventional node.
    boost::recursive_mutex configMutex;
    //std::shared_ptr<dynamic_reconfigure::Server<kugle_driver::ParametersConfig>> reconfigureServer = std::make_shared<dynamic_reconfigure::Server<kugle_driver::ParametersConfig>>(configMutex);
    reconfigureServer = std::make_shared<dynamic_reconfigure::Server<jetsoncar_driver::ParametersConfig>>(configMutex);
    dynamic_reconfigure::Server<jetsoncar_driver::ParametersConfig>::CallbackType f;
    f = boost::bind(&ROS_ReconfigureCallback, _1, _2, lspcMutex, lspcObj);
    reconfigureServer->getConfigDefault(reconfigureConfig);
    reconfigureMutex.lock();
    reconfigureServer->setCallback(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // wait for initial reconfigure request
    reconfigureMutex.unlock();

	/* Create LSPC connection thread */
    std::promise<void> exitSignal;
    std::future<void> exitSignalObj = exitSignal.get_future();
    std::thread connectionThread = std::thread(&LSPC_ConnectionThread, n, serial_port, lspcMutex, lspcObj, std::move(exitSignalObj));

    ros::spin();

    ROS_DEBUG("Shutting down JetsonCar driver");

    //reference_timeout_timer->stop();

    exitSignal.set_value();
    if (connectionThread.joinable())
        connectionThread.join();
}
