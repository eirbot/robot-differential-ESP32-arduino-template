/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef ODOMETRY_EIRBOT_H
#define ODOMETRY_EIRBOT_H

#include "odometry/odometry_differential.h"
#include <Arduino.h>

namespace sixtron {

class OdometryEirbot: public OdometryDifferential {

public:
    OdometryEirbot(float rate_hz,
            uint32_t sensor_resolution,
            float motor_resolution,
            float wheel_radius,
            float wheels_distance):
            OdometryDifferential(rate_hz, motor_resolution, wheel_radius, wheels_distance),
            _sensor_resolution(sensor_resolution) {};

    ~OdometryEirbot() = default;

    // Specific for eirbot mobile base.
    void init() override;

    // Specific for eirbot rolling base
    void update() override;

private:
    static void update_encoder_counter(uint16_t current_ticks,
            uint16_t previous_ticks,
            int32_t *revolution_counter,
            int64_t *global_ticks_counter,
            uint32_t sensor_resolution);

    uint32_t _sensor_resolution;
    uint16_t _previous_raw_tick_left = 0.0f, _previous_raw_tick_right = 0.0f;
    int32_t _sensor_revolution_left = 0.0f, _sensor_revolution_right = 0.0f;
    int64_t _counter_tick_left = 0.0f, _counter_tick_right = 0.0f;
};
}

#endif // ODOMETRY_EIRBOT_H
