/*
 * Copyright (c) 2025, EIRBOT
 * SPDX-License-Identifier: Apache-2.0
 */

#include "robot_eirbot.h"

// CONTROL LOOP (Mbed THREAD in real time)
#define CONTROL_THREAD_RATE_MS 50 // 50ms
// #define CONTROL_THREAD_FLAG 0x02

// ticker object
hw_timer_t *_hw_timer = NULL;

// RBDC class instantiation
sixtron::MobileBaseEirbot *mobile_base;
sixtron::OdometryEirbot *odom;
sixtron::RBDC *rbdc;

const std::string rbdc_status[RBDC_MAX_STATUS] = {
    "RBDC_standby",
    "RBDC_working",
    "RBDC_done",
    "RBDC_correct_f_angle",
    "RBDC_moving",
    "RBDC_moving_&_angle",
    "RBDC_correct_i_angle",
};

volatile int rbdc_result = sixtron::RBDC_status::RBDC_standby;
volatile uint8_t task_update = 0; // uint8_t faster than bool ? to be confirmed

/* ######################  GOTO FUNCTIONS   ############################################ */

// Be aware this is a blocking function by default
void robot_goto(float x, float y, float theta, bool blocking, sixtron::RBDC_reference reference) {
    rbdc->setTarget(x, y, theta, reference);
    delay(100); // let time for the control loop to update
    if (blocking) {
        while (rbdc_result != sixtron::RBDC_status::RBDC_done) {
            delay(10);
        }
    }
}

// Just go to X/Y, no final theta
void robot_goto(float x, float y, bool blocking, sixtron::RBDC_reference reference) {
    rbdc->setTarget(x, y, reference);
    delay(100);
    if (blocking) {
        while (rbdc_result != sixtron::RBDC_status::RBDC_done) {
            delay(10);
        }
    }
}

/* ######################  RBDC CONTROL LOOP   ############################################ */

// This function is called periodically by an ESP ticker, and will set a flag.
void controlThreadUpdate() {
    task_update = 1;
}

void control(void *pvParameters) {

    terminal_printf("[RBDC] Init ...\n");

    // Convert current rate of the loop in seconds (float)
    float dt_pid = float(CONTROL_THREAD_RATE_MS) / 1000.0f; // Very important for all PIDs
    float hz_pid = 1.0f / dt_pid;

    // Setup and init odometry (this will init steppers communication)
    // In Eirbot case, for stepper in direct drive, the motor resolution is the same as sensor.
    odom = new sixtron::OdometryEirbot(
            hz_pid, SENSOR_RESOLUTION, (float)SENSOR_RESOLUTION, WHEEL_RADIUS, WHEELS_DISTANCE);
    odom->init();

    // Setup and init the motor base
    mobile_base = new sixtron::MobileBaseEirbot(WHEELS_DISTANCE);
    mobile_base->init();

    // Setup RBDC
    // TODO: update the values !!
    sixtron::RBDC_params rbdc_params;
    rbdc_params.rbdc_format = sixtron::RBDC_format::two_wheels_robot;
    rbdc_params.max_output_dv = 1.0f; // in [m/s]
    rbdc_params.max_output_dtheta = 8.0f; // in [rad/s]
    rbdc_params.can_go_backward = true;
    rbdc_params.dt_seconds = dt_pid;
    rbdc_params.final_theta_precision = DEG_TO_RAD(3);
    rbdc_params.moving_theta_precision = 10 * ONE_DEGREE_IN_RAD;
    rbdc_params.target_precision = 3 * PID_DV_PRECISION;
    rbdc_params.dv_precision = PID_DV_PRECISION;

    // Theta, or angular speed, PID parameters
    // TODO: update the values !!
    rbdc_params.pid_param_dteta.Kp = 1.0f;
    rbdc_params.pid_param_dteta.Ki = 0.0f;
    rbdc_params.pid_param_dteta.Kd = 0.0f;
    rbdc_params.pid_param_dteta.ramp = 1.75f;

    // Dv, or linear speed, PID parameters
    // TODO: update the values !!
    rbdc_params.pid_param_dv.Kp = 2.0f;
    rbdc_params.pid_param_dv.Ki = 0.1f;
    rbdc_params.pid_param_dv.Kd = 0.25f;
    // rbdc_params.pid_param_dv.ramp = 0.5f;

    rbdc = new sixtron::RBDC(odom, mobile_base, rbdc_params);

    // setup ESP32 timer ticker to periodically call "controlThreadUpdate" function
    _hw_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(_hw_timer, controlThreadUpdate, true);
    // second value must be in us, because timer divide ESP32 clock (80Mhz) by 80 so it's a us clock
    timerAlarmWrite(_hw_timer, CONTROL_THREAD_RATE_MS * 1000, true);
    timerStart(_hw_timer);
    timerAlarmEnable(_hw_timer);

    terminal_printf("[RBDC] Init done.\n");
    float time_passed = 0.0f;

    while (true) {

        // todo: there should be a better way to wait for a flag, but can't find it on freertos...
        // Wait for asserv tick
        while (task_update == 0) {
            delay(1);
        }

        // reset flag
        task_update = 0;

        // check if we can move or not
        if (robot_can_move) {
            rbdc->start();
        } else {
            rbdc->pause();
        }

        // Update RBDC (will automatically update odometry, motor base, QEI, motors, PIDs...)
        rbdc_result = rbdc->update();

        // update time counter for next round (for debug purposes)
        time_passed += dt_pid;
    }
}

// MBED STARTING NEW THREAD
void start_eirbot_rbdc_control_thread() {

    xTaskCreate(control, /* Task function. */
            "robot_control_loop", /* name of task. */
            8192, /* Stack size of task */
            NULL, /* parameter of the task */
            1, /* priority of the task */
            NULL); /* Task handle to keep track of created task */

    delay(1000); // wait a bit before continue
}
