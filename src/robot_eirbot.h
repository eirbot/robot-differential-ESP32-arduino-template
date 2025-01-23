/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_EIRBOT_H
#define ROBOT_EIRBOT_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "RBDC/RBDC.h"
#include "common.h"
#include "cstring"
#include "mobile_base_eirbot.h"
#include "odometry_eirbot.h"

// radian degree conversion
#define ONE_DEGREE_IN_RAD 0.017453f // 1°
#define ONE_RAD_IN_DEG 57.2958f // 1rad
#define DEG_TO_RAD(x) ((x)*ONE_DEGREE_IN_RAD)

// Robot and control parameters
// TODO: Update the parameters values !!!
#define SENSOR_RESOLUTION 0xFFFF
#define WHEEL_RADIUS 0.036f // 3.6 cm
#define WHEELS_DISTANCE 0.328f // distance between the two wheels of the differential robot
#define PID_TETA_PRECISION DEG_TO_RAD(10) // 10° of precision
#define PID_DV_PRECISION 0.05f // 5 cm of precision

void robot_goto(float x,
        float y,
        float theta,
        bool blocking = true,
        sixtron::RBDC_reference reference = sixtron::RBDC_reference::absolute);

void robot_goto(float x,
        float y,
        bool blocking = true,
        sixtron::RBDC_reference reference = sixtron::RBDC_reference::absolute);

void start_eirbot_rbdc_control_thread();

#endif // ROBOT_EIRBOT_H
