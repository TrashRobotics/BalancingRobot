<h1 align="center">
  <a href="link to video"><img src="https://github.com/TrashRobotics/BalancingRobot/blob/main/img/bbot.jpg" alt="Automatic tumbler" width="800"></a>
  <br>
  Self-balancing robot based on NodeMCU
  <br>
</h1>

<p align="center">
  <a href="https://github.com/TrashRobotics/BalancingRobot/blob/main/README.md">Русский</a> •
  <a href="https://github.com/TrashRobotics/BalancingRobot/blob/main/README-en.md">English(Английский)</a> 
</p>

# Description

Self-balancing robot for fun is capable of:
* actually, to balance; :)
* to ride and turn;
* to deviate from obstacles using distance sensors;
* all of it are controlled through an access point distributed by the robot itself, which acts, among other things, as a web server, issuing a control page to a standard browser upon request.

# Main parts
* esp8266 NodeMCU v3;
* motor driver l298n;
* yellow single-axis arduino TT-geared motors;
* 2x 18650 batteries and battery compartment;
* MPU6050;
* SSD1306 128x64 I2C display;
* 2x HC-SR04 distance sensors;
* toggle switch KCD1-11;
* breadboard 4x6 см (if you'd like);
* body parts (добавить ссылку на модели stl) 

![Main parts](https://github.com/TrashRobotics/BalancingRobot/blob/main/img/parts.jpg)

# Wiring diagram
![Wiring diagram](https://github.com/TrashRobotics/BalancingRobot/blob/main/img/schematic.jpeg)

# Installation and uploading
To compile the sketch, you need to install the following libraries:
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson);
* [L298N](https://github.com/AndreaLombardo/L298N);
* [MPU6050](https://github.com/ElectronicCats/mpu6050);
* [Ultrasonic](https://github.com/ErickSimoes/Ultrasonic);
* [ThingPulse SSD1306](https://github.com/ThingPulse/esp8266-oled-ssd1306);

Libraries are installed by following links and clicking **Code->Download ZIP**.            
В Arduino IDE **Sketch->Include library->Add .ZIP library...**

After that you need to download the balancing_robot directory and put it in the Arduino IDE workspace. (sources.h should be in the same folder with the sketch).

Compile and upload.

# Start and connection 
When the power is turned on, the robot will start balancing.    
To control it, you need to connect to an access point :

```
ssid: balancingbot;   // default
psw: 123456789;
```
and enter the robot's ip-address in the browser:
```
192.168.1.1   // default
```
have fun :)
