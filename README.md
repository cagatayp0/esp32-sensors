# An Example of ESP32 (Not Arduino) Programming

Most of the tutorials online contain information about Arduino, while there are not any detailed instructions about ESP32. Furthermore, there are a lot of junky codes on the internet which compiles successfully but doesn't run optimal. In this repository, I tried to gather some valuable information on the internet about ESP32 such as:

* Dual Core Programming with FreeRTOS
* I2C Bus
* UART Hardware Serial (Instead of Software Serial on Arduino)
* Google Firebase Integration (Will be included)
* Hybernation Mode (Will be included)

## The Project

This project aims to gather data from an imu sensor (Sparkfun 9dof Stick), an air quality sensor (DFRobot BME680) & a gps module (U-Blox Neo6m) and to send them to a database (Google Firebase). Imu and air quality sensors are communicating with the ESP32 via I2C protocol, whereas gps module sends bits with UART protocol. I used NodeMCU 32-S as development card. The following image shows the wiring.

## Most Common Problems I've Faced When I'm Working on This Project

### Using Software Serial Instead of Hardware Serial

### Not Using Dual Core of ESP32

## References

* GPRMC Decoding by David Watts: https://www.youtube.com/watch?v=bgOZLgaLa0g&t=485s
* I2C Bus Explanation: https://github.com/SensorsIot/I2C_1-Video
* Dual Core Programming Example: https://github.com/RalphBacon/ESP32-Variable-Passing-Using-Queues-Global-Vars
* Sparkfun 9dof Stick repository: https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
* DFRobot BME680 repository: https://github.com/DFRobot/DFRobot_BME680
