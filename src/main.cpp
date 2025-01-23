/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#include "robot_eirbot.h"
#include <Arduino.h>

void setup() {

    // Setup the serial
    Serial.begin(115200);
    terminal_printf("[MAIN] Init ...\n");

    // Setup Eirbot RBDC thread
    terminal_printf("Setup EIRBOT robot base control thread for a differential robot\n");
    start_eirbot_rbdc_control_thread();

    // enable robot moves at start
    robot_can_move = true;
    // include common.h to your lidar detection thread, to edit global "robot_can_move" variable

    terminal_printf("[MAIN] Init Done.\n");
}

void loop() {

    // Do the square indefinitely
    robot_goto(1.0, 0.0);
    robot_goto(1.0, 1.0);
    robot_goto(0.0, 1.0);
    robot_goto(0.0, 0.0);

    // Wait 5 sec and do another square
    delay(5000); // in ms
}
