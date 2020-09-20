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

/* OBS. Remember to keep this file consistent across the Embedded Firmware and ROS Driver */

#ifndef LSPC_MESSAGE_TYPES_HPP
#define LSPC_MESSAGE_TYPES_HPP

#include <cstdint>

namespace lspc
{
    namespace MessageTypesFromPC
    {
        typedef enum MessageTypesFromPC: uint8_t
        {
            Test = 0x01,
            SetPID = 0x02,
            Setpoint = 0x03,
            EnterBootloader = 0xF0,
            Reboot = 0xF1,
            Debug = 0xFF
        } MessageTypesFromPC_t;

        typedef struct
        {
            float P;
            float I;
            float D;
        } SetPID_t;

        typedef struct
        {
            float angular_velocity;
            float steering;
        } Setpoint_t;

        typedef struct
        {
            uint32_t magic_key;
        } EnterBootloader_t;

        typedef struct
        {
            uint32_t magic_key;
        } Reboot_t;
    }

    namespace MessageTypesToPC
    {
        typedef enum MessageTypesToPC: uint8_t
        {
            Test = 0x01,
            Sensors = 0x02,
            CPUload = 0xE1,
            Debug = 0xFF
        } MessageTypesToPC_t;

        typedef struct
        {
            float timestamp;
            struct wheel_angles_t
            {
                float front;
                float rear;
            } wheel_angles;
            struct wheel_angular_velocities_t
            {
                float front;
                float rear;
            } wheel_angular_velocities;
            struct rc_t
            {
                float throttle;
                float steering;
            } rc;
            struct setpoints_t
            {
                float angular_velocity;
                float steering;
            } setpoints;
            struct motor_outputs_t
            {
                float throttle;
                float steering;
            } motor_outputs;
        } Sensors_t;
    }

} // namespace lspc

#endif // LSPC_MESSAGE_TYPES_HPP
