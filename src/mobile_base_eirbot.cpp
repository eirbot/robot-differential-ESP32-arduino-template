/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "mobile_base_eirbot.h"

namespace sixtron {

void MobileBaseEirbot::init() {
    // TODO: Be sure that motors are working and have a speed of 0m/s
    //   - Communication is already setup in odometry !
    //   - set the motor to 0ms

    set_stepper_speed(motor_side::motor_left, 0.0f);
    set_stepper_speed(motor_side::motor_right, 0.0f);
}

void MobileBaseEirbot::update() {
    // compute the speeds for a differential robot, see "mobile_base_differential.cpp"
    computeMotorSpeeds();

    // TODO: send speeds to the steppers, in meter per seconds
    set_stepper_speed(motor_side::motor_left, _targetSpeedMotorLeft);
    set_stepper_speed(motor_side::motor_right, _targetSpeedMotorRight);

    // Could be useful in the future for a Lidar.
    if ((_targetSpeedMotorLeft > 0.0f && _targetSpeedMotorRight < 0.0f)
            || (_targetSpeedMotorLeft < 0.0f && _targetSpeedMotorRight > 0.0f)) {
        _running_side = TURNING_ON_ITSLEF;
    } else if (_targetSpeedMotorLeft >= 0.0f && _targetSpeedMotorRight >= 0.0f) {
        _running_side = RUNNING_FRONT;
    } else if (_targetSpeedMotorLeft < 0.0f && _targetSpeedMotorRight < 0.0f) {
        _running_side = RUNNING_BACK;
    } else {
        _running_side = NOT_MOVING;
    }

    // Check if the motor should be powered or not
    if (_latest_state != _mobile_base_status) {
        _latest_state = _mobile_base_status;

        if (_mobile_base_status == MobileBase_status::mobile_base_start) {
            // TODO: start the motor (power up)
            start_stepper(motor_side::motor_left);
            start_stepper(motor_side::motor_right);
        } else if (_mobile_base_status == MobileBase_status::mobile_base_stop) {
            // TODO: stop the motor (power down)
            stop_stepper(motor_side::motor_left);
            stop_stepper(motor_side::motor_right);
        }
    }
}

int MobileBaseEirbot::get_running_side() {
    return _running_side;
}

float MobileBaseEirbot::get_speed_left_target() {
    return _targetSpeedMotorLeft;
}

float MobileBaseEirbot::get_speed_right_target() {
    return _targetSpeedMotorRight;
}

}
