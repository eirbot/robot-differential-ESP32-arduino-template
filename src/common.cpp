/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common.h"

// exemple for lidar detection
bool volatile robot_can_move;

// custom printf
void terminal_printf(const char *fmt, ...) {
    static char terminal_buff[MAX_PRINTF_LENGTH];
    va_list args;
    va_start(args, fmt);
    int length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    Serial.write(terminal_buff, length);
}

void terminal_debug(const char *fmt, ...) {
#if DEBUG_ENABLE
    static char terminal_buff[MAX_PRINTF_LENGTH];
    va_list args;
    va_start(args, fmt);
    int length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    Serial.write(terminal_buff, length);
#endif
}
