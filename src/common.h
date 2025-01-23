/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef COMMON_H
#define COMMON_H

#define DEBUG_ENABLE 0
#define MAX_PRINTF_LENGTH 100
#include "string.h"
#include <Arduino.h>

extern volatile bool robot_can_move;

// uncomment if you want to change the default serial for terminal printf
// HardwareSerial terminal(2); // keep here
// terminal.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // do that in main setup

// custom printf
void terminal_printf(const char *fmt, ...);
void terminal_debug(const char *fmt, ...);

#endif
