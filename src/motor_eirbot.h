/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_EIRBOT_H
#define MOTOR_EIRBOT_H

#include <Arduino.h>

typedef enum {
    motor_left,
    motor_right,
} motor_side;

void init_steppers_communication();
void reset_steppers_encoder();

void start_stepper(motor_side motor);
void stop_stepper(motor_side motor);

void set_stepper_speed(motor_side motor, float speed);
uint16_t get_stepper_encoder(motor_side motor);

#endif // MOTOR_EIRBOT_H
