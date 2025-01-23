/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_eirbot.h"

void init_steppers_communication() {
    // TODO: Init the serial or I2C communication here !
}

void reset_steppers_encoder() {
    // TODO: Only if possible, this function should communicate with
    //  stepper boards to reset encoder values.
}

void start_stepper(motor_side motor) {
    // TODO: start the motor (power up, the stepper can turn)
}

void stop_stepper(motor_side motor) {
    // TODO: stop the motor (power down, the stepper is not moving, and won't turn)
    //  if it can't be done, apply a speed of 0.0ms, to completely stop the motor.
}

void set_stepper_speed(motor_side motor, float speed) {
    // TODO: communicate with the stepper board to apply a speed, in meters per second !
    //  (The RBDC will send you speed in ms, so you may need to do the conversion rpm -> ms)

    if (motor == motor_side::motor_left) {
        // TODO: set speed for motor left
    } else if (motor == motor_side::motor_right) {
        // TODO: set speed for motor right
    }
}

uint16_t get_stepper_encoder(motor_side motor) {
    // TODO: communicate with the stepper board to get the current encoder value !
    //  Value should be between 0x0000 and max 0xFFFF, and not exceed the defined
    //  "sensor_resolution" value in odometry_eirbot!

    float encoder = 0;

    if (motor == motor_side::motor_left) {
        // TODO: get current encoder value for motor left
        //  encoder = ...
    } else if (motor == motor_side::motor_right) {
        // TODO: get current encoder value for motor right
        //  encoder = ...
    }

    return encoder;
}
