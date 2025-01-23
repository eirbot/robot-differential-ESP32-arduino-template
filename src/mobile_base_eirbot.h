/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

// ============ THIS FILE IS APPLICATION SPECIFIC ========================

#ifndef MOBILE_BASE_EIRBOT_H
#define MOBILE_BASE_EIRBOT_H

#include "mobile_base/mobile_base_differential.h"
#include "motor_eirbot.h"
#include <Arduino.h>

namespace sixtron {

#define NOT_MOVING 0
#define RUNNING_FRONT 1
#define RUNNING_BACK 2
#define TURNING_ON_ITSLEF 3

class MobileBaseEirbot: public MobileBaseDifferential {

public:
    MobileBaseEirbot(float wheels_distance): MobileBaseDifferential(wheels_distance) {};

    ~MobileBaseEirbot() = default;

    // Specific for eirbot mobile base.
    void init() override;

    // Specific for eirbot mobile base.
    void update() override;

    // Could be useful for a Lidar.
    int get_running_side();

    // For debug purposes, get RBDC calculated speed for each motor
    float get_speed_left_target();
    float get_speed_right_target();

private:
    MobileBase_status _latest_state;
    int _running_side;
};
}

#endif // MOBILE_BASE_EIRBOT_H
