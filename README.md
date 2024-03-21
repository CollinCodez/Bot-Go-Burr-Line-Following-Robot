# Bot Go Burr Line Following Robot Code
This is the code designed for our Mechatronics Engineering Capstone Project, a Line Following Robot.

## Features
Some nice features about our implementation include:
- A wireless web interface to control the robot, allowing for live monitoring and tuning of PID constants.
- A few `#define` based configuration options, to easily remove some parts of the code or change running modes for the code. These included:
  - `SERIAL_ENABLED` - Setting to 0 will remove all serial printing lines from the code
  - `NO_WIRELESS` - Setting to 1 will remove all wireless functionality from the code and greatly simplify the startup process. This will make the bot automatically begin calibration on power up, then start the main control loop, with no wireless functionality at all. This was added for reliability in case we were having issues with connectivity to the web interface, and reduced the number of tasks running on the ESP32.
  - `ENABLE_EDF` - Setting to 0 removes all ducted fan functionality from the code, setting it to 1 will use the DShotRMT library to control the ducted fan, and setting it to 2 will instead use an ESC library that utilizes the servo-like PWM control method (This was tested, but we did not have success with getting this library to work).
  - `HALF_SENSORS` - Setting this to 1 will make the robot use only every-other sensor, making the sensor act like an 8mm spaced, 8 sensor array. This was tested some for potentially faster and more reliable readings of the sensor, to allow for faster loop times and better control of the robot.
- The addition of a Proportional<sup>2</sup> term in the PID control. This allows for hard changes near the ends of the sensor, while allowing the bot to have a sort of deadzone when going straight. 

## Resources
A number of different guides and libraries were used to make this project possible
- Rui Santos's numerous [ESP32 tutorials](https://randomnerdtutorials.com/projects-esp32/)
  - These were EXTREMELY helpful for the development of the web interface, along with numerous other aspects of working with an ESP32.
- [ESPAsyncWebServer](https://github.com/lacamera/ESPAsyncWebServer)
  - Easy way to implement WebSockets for simple communication between the bot and the web interface, while also not interrupting the main control loop's code
- [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA)
  - Extremely simple way to add the ability to wirelessly update the firmware on the robot (requiring only 2 lines of code to be added), while also simply piggybacking on the already implemented ESPAsyncWebServer and adding minimal overhead.
- [ArduinoJSON](https://github.com/bblanchon/ArduinoJson)
  - Simple to use JSON library that is much more more efficient than the "official" [Arduino_JSON](https://github.com/arduino-libraries/Arduino_JSON/) library
- Pololu's [QTRSensors](https://github.com/pololu/qtr-sensors-arduino) Library
  - Easy to implement library designed for our Pololu [QTR-HD-15RC Reflectance Sensor Array](https://www.pololu.com/product/4115)
- Carbon225's [ESP32 DSHOT](https://github.com/Carbon225/esp32-dshot) Library
  - Easy to use, effective DSHOT library for the ESP32, which takes advantage of the RMT Channel outputs of the ESP32, reducing the load on the processor. This was used for the control of our ducted fan.
