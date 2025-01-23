/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#include "odometry_eirbot.h"
#include "motor_eirbot.h"

namespace sixtron {

// Call only one time (automatically by the RBDC)
void OdometryEirbot::init() {

    // TODO: Init the motors here !
    //  - Setting up the communication to the motor board
    //  - reset all encoder values (IF POSSIBLE). But if stepper encoder start at 0 at reboot, it's
    //  should be ok.

    init_steppers_communication();
    reset_steppers_encoder(); // encoders must start at 0 when robot is booting up.
}

void OdometryEirbot::update() {

    // TODO: This function will get the current encoders values, check the delta, and
    //  compute the differential odometry through "OdometryDifferential" class

    uint16_t raw_tick_left = get_stepper_encoder(motor_side::motor_left);
    uint16_t raw_tick_right = get_stepper_encoder(motor_side::motor_right);

    // Check for delta, or "effet de bord", and update each encoder global counter
    update_encoder_counter(raw_tick_left,
            _previous_raw_tick_left,
            &_sensor_revolution_left,
            &_counter_tick_left,
            (int32_t)_sensor_resolution);
    update_encoder_counter(raw_tick_right,
            _previous_raw_tick_right,
            &_sensor_revolution_right,
            &_counter_tick_right,
            (int32_t)_sensor_resolution);

    // update previous ticks for next update
    _previous_raw_tick_left = raw_tick_left;
    _previous_raw_tick_right = raw_tick_right;

    // This will launch the calculation for differential odometry, based on the two stepper counters
    compute(_counter_tick_left, _counter_tick_right);
}

// TODO: CHeck if the following function is working correctly !!

// internal function to check for encoder delta
void OdometryEirbot::update_encoder_counter(uint16_t current_ticks,
        uint16_t previous_ticks,
        int32_t *revolution_counter,
        int64_t *global_ticks_counter,
        uint32_t sensor_resolution) {

    int32_t countDelta = (int16_t)(current_ticks) - (int16_t)(previous_ticks);

    // Check if we did an overflow
    if (countDelta >= (sensor_resolution / 2)) {
        *revolution_counter = *revolution_counter - 1;
    } else if (countDelta <= -(sensor_resolution / 2)) {
        *revolution_counter = *revolution_counter + 1;
    }

    // update the global counter
    *global_ticks_counter
            = ((*revolution_counter) * ((int64_t)sensor_resolution)) + (int64_t)current_ticks;
}

}
