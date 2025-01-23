# Robot Differential FreeRTOS Template

This is a template to create a RBDC project for a differential robot, using PlatformIO and FreeRTOS framework, on es^ressif32 (ESP32) platform.

> [!NOTE]
>
> The RBDC stand for "Robot Base Drive Control". It's a library to control a robot from an embedded target (like a STM32 on a NUCLEO-F446RE), regarding the number of wheels or the type of odometry.
>
> The user has to properly "connect" the RBDC to his hardware, then periodically update the RBDC to update the odometry and the motor target speeds.
>
> It's still under development, main library repository can be found here: [https://github.com/catie-aq/6tron_RBDC](https://github.com/catie-aq/6tron_RBDC).



> [!WARNING]
>
> You will need to use CPP17, hence the build flags changes in `platformio.ini`. (or else the RBDC can't be build. [Issue to investigate](https://github.com/catie-aq/6tron_RBDC/issues/65))
