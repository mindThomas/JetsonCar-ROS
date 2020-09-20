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

#include <jetsoncar_driver/jetsoncar_driver_node/lspc_callbacks.h>

#include "LSPC.h"
#include "MessageTypes.h"

#include "jetsoncar_driver/odometry.h"

// For CLion to update/capture the changes made to generated services, message types and parameters, open the "build" folder and run "make"
/* Include generated Message Types */
#include <jetsoncar_msgs/Encoders.h>

/* Include messages */
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

void ROS_PublishWheelTransforms(tf::TransformBroadcaster& tfBroadcaster, const lspc::MessageTypesToPC::Sensors_t& msg);

void LSPC_Callback_Sensors(ros::Publisher& pubEncoders, tf::TransformBroadcaster& tfBroadcaster, ros::Publisher& pubOdom, const std::vector<uint8_t>& payload)
{
    static float prev_timestamp = 0;

    const lspc::MessageTypesToPC::Sensors_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::Sensors_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing Sensors message");
        return;
    }

    jetsoncar_msgs::Encoders msg;
    msg.receive_time = ros::Time::now();
    msg.mcu_time = msgRaw->timestamp;
    msg.front_angle = msgRaw->wheel_angles.front;
    msg.rear_angle = msgRaw->wheel_angles.rear;
    msg.front_velocity = msgRaw->wheel_angular_velocities.front;
    msg.rear_velocity = msgRaw->wheel_angular_velocities.rear;
    msg.steering_angle = msgRaw->motor_outputs.steering;
    pubEncoders.publish(msg);

    if (prev_timestamp > 0) {
        double wheel_velocity = (msgRaw->wheel_angular_velocities.front + msgRaw->wheel_angular_velocities.rear) / 2.0;

        // Compute necessary variables
        float dt = msgRaw->timestamp - prev_timestamp;
        double v = wheel_radius * wheel_velocity; // translational velocity
        double rho = msgRaw->motor_outputs.steering * max_steering; // convert steering percentage to steering angle

        PublishOdometry(v, rho, dt, msg.receive_time, tfBroadcaster, pubOdom);
    }

    ROS_PublishWheelTransforms(tfBroadcaster, *msgRaw);

    prev_timestamp = msgRaw->timestamp;
}

void ROS_PublishWheelTransforms(tf::TransformBroadcaster& tfBroadcaster, const lspc::MessageTypesToPC::Sensors_t& msg)
{
    geometry_msgs::TransformStamped tf;
    tf::Quaternion q;
    tf.header.stamp = ros::Time::now();
    tf.transform.translation.x = 0;
    tf.transform.translation.y = 0;
    tf.transform.translation.z = 0;

    /* Send transform from "front_right_hinge" frame to "front_right_wheel" frame */
    q = tf::createQuaternionFromRPY(0, msg.wheel_angles.front, msg.motor_outputs.steering * max_steering); // rotate around z-axis
    tf.header.frame_id = "front_right_hinge";
    tf.child_frame_id = "front_right_wheel";
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tfBroadcaster.sendTransform(tf);

    /* Send transform from "front_left_hinge" frame to "front_left_wheel" frame */
    q = tf::createQuaternionFromRPY(0, msg.wheel_angles.front, msg.motor_outputs.steering * max_steering); // rotate around z-axis
    tf.header.frame_id = "front_left_hinge";
    tf.child_frame_id = "front_left_wheel";
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tfBroadcaster.sendTransform(tf);

    /* Send transform from "front_shaft" frame to "front_left_hinge" frame */
    tf.header.frame_id = "front_shaft";
    tf.child_frame_id = "front_left_hinge";
    tf.transform.translation.y = left_to_right_wheel_distance / 2;
    tf.transform.rotation.w = 1;
    tf.transform.rotation.x = 0;
    tf.transform.rotation.y = 0;
    tf.transform.rotation.z = 0;
    tfBroadcaster.sendTransform(tf);

    /* Send transform from "front_shaft" frame to "front_right_hinge" frame */
    q = tf::createQuaternionFromRPY(0, msg.wheel_angles.rear, 0); // rotate around z-axis
    tf.header.frame_id = "front_shaft";
    tf.child_frame_id = "front_right_hinge";
    tf.transform.translation.y = -left_to_right_wheel_distance / 2;
    tf.transform.rotation.w = 1;
    tf.transform.rotation.x = 0;
    tf.transform.rotation.y = 0;
    tf.transform.rotation.z = 0;
    tfBroadcaster.sendTransform(tf);

    /* Send transform from "rear_left_hinge" frame to "rear_left_wheel" frame */
    q = tf::createQuaternionFromRPY(0, msg.wheel_angles.rear, 0); // rotate around z-axis
    tf.header.frame_id = "rear_shaft";
    tf.child_frame_id = "rear_left_wheel";
    tf.transform.translation.y = left_to_right_wheel_distance / 2;
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tfBroadcaster.sendTransform(tf);

    /* Send transform from "rear_right_hinge" frame to "rear_right_wheel" frame */
    q = tf::createQuaternionFromRPY(0, msg.wheel_angles.rear, 0); // rotate around z-axis
    tf.header.frame_id = "rear_shaft";
    tf.child_frame_id = "rear_right_wheel";
    tf.transform.translation.y = -left_to_right_wheel_distance / 2;
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tfBroadcaster.sendTransform(tf);
}

void LSPC_Callback_CPUload(ros::Publisher& pubLoad, const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Microprocessor CPU Load\n" << message);

    /* Publish data to topic as well */
    std_msgs::String msg;
    msg.data = "\n" + message;
    pubLoad.publish(msg);
}

void LSPC_Callback_ArrayDump(std::shared_ptr<std::ofstream> log_file, const std::vector<uint8_t>& payload)
{
    int numberOfFloats = payload.size() / 4;
    const float * floatArray = reinterpret_cast<const float *>(payload.data());

    if (log_file->is_open()) {
        for (int i = 0; i < numberOfFloats; i++) {
            if (i > 0) *log_file << "\t";
            *log_file << std::setprecision(10) << floatArray[i];
        }
        *log_file << std::endl;
    }
}

void LSPC_Callback_Debug(ros::Publisher& pubDebug, const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Debug message:\n" << message);

    /* Publish data to topic as well */
    std_msgs::String msg;
    msg.data = "\n" + message;
    pubDebug.publish(msg);
}