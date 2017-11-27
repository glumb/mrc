<img src="https://cloud.githubusercontent.com/assets/3062564/24917600/fb3c518c-1edd-11e7-9755-6fff083a112b.png" alt="mrc-logo" width=70 />

# mrc

[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/glumb/mrc/master/LICENSE.md)
[![Travis](https://img.shields.io/travis/glumb/mrcl.svg)](https://travis-ci.org/glumb/mrc)

MicroPede Robot Controller 🤖🖥👾

The MRC lets you control 6 DOF/axis robotic arms using the g-code like [MRIL](https://github.com/glumb/mrcp) language.
The istructions can either be transferred directly to the controller using `screen`, or any other serial monitor, or with the help of a library, such as the [MRCL](https://github.com/glumb/mrcl) (MicroPede Robot Control Library)

Zu den Robotern [hier entlang](https://micropede.de/shop#filter=.robot).

This project is work in progress.
Feel free to fork, update and improve it 😃

All configurations and settings are based on the robot model depiced below.

<img width="300" alt="bildschirmfoto 2017-05-15 um 12 11 43" src="https://cloud.githubusercontent.com/assets/3062564/26053066/b188e2ea-3967-11e7-8a45-2697041ca6a6.png"><img width="400" alt="bildschirmfoto 2017-05-15 um 12 12 08" src="https://cloud.githubusercontent.com/assets/3062564/26053074/bc6a01d0-3967-11e7-83c9-d572eef1bd12.png">
<img width="603" alt="bildschirmfoto 2017-05-15 um 12 12 45" src="https://cloud.githubusercontent.com/assets/3062564/26053120/e27b9e38-3967-11e7-8cd5-38c0fa0669ca.png">


## Getting started
Clone the repo

```shell
git clone https://github.com/glumb/mrc.git
```


Set you microcontroller in the platformio.ini.

Uncomment `#define EXAMPLES 1` in the `main.cpp` file to build the calibration example.

In `CalibrateServos.h` set the `pin_servo_n` according to your setup.

Build the project.

```shell
platformio run
```

Upload the program to the microcontroller.

```shell
platformio run --target upload
```

Calibrate your servos (see section Calibration).

Change the configuration to match your robot (see section Configuration).

Optionally attach a 128x64 display to the pins defined in `main.cpp`

Comment `#define EXAMPLES 1`, then build and upload the project.

Use the [MRCP/MRIL](https://github.com/glumb/mrcp) to control the robot.

Send a few commands via serial to check if everything is working. `B` should return the free buffer size (300).


## PlatformIO

This project was developed with the help of [PlatformIO](http://platformio.org) (PIO). PIO is an open source cross-platform build system and Atom based IDE. It is used to build and upload programs to a microcontroller.
I highly recommend using the PIO IDE or at least the PIO build-system to upload the program to your Arduino/Teensy.


## Configuration

Most configuration is located in the `config.h` file.

Aspects that have to be configured:

| Config                     | Description                                                                                                                                                                                                                                                                                                                                                                               |
|----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Servo Pins                 | Set which PWM pins you want to use on your μC for driving the servos                                                                                                                                                                                                                                                                                                                      |
| Servo Config               | Assign pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax and home position to each servo. This configuration can be generated using the `CalibrateServos.h` in the examples directory.                                                                                                                                                                    |
| Additional Axis Pins       | Set which PWM pins to use for additional Servos. (e.g. grippers)                                                                                                                                                                                                                                                                                                                          |
| Geometry                   | Configure the robot geometry based on the convention depicted above. The base coordinate system (left, bottom) is used as reference. V3 must have no offset in Y, Z. V4 must have no offset in X, Y.                                                                                                                                                                                      |
| Pin Map                    | The PinMap is used for setting the IO pins, used in the MRIL I, O commands (I2 1 wait for pin 2:high, O7 0 set pin 7:low).  A PinMap {8, 3, 24}, maps the physical pins 8, 3, 24 to the logical pins 0, 1, 2.                                                                                                                                                                             |
| Logical to Physical Angles | A method to transform logical to physical angles. (See section kinematics for more detail.) |

## Servo Calibration
The servos have to be calibrated to relate PWM pulses to physical robot positions. Therefore build and upload the `/examples/CalibrateServos.h` file.
Open a serial monitor and connect to the microcontroller.
Use the keys `0-9 to select a servo`. Press `n to move` a servo without changing the config.
`w increments`, `s decrements` the current value.

## Workflow
You basically alter the servo frequency to move the servos to certain positions and then relate those positions to physical angles. You can always press **h** to display all commands and explanations.

+ select a servo using keys **0-9**
+ press **n** to set rapid movement mode
+ move the servo to the maximum position (using **w**,**s**) according to the joint rotation direction depicted above.
+ press **z** to enter "set maximum PWM value" mode.
+ again use using **w**,**s** for small adjustments. Presseing w,s now sets the maximum PWM value
+ use a ruler or similar device to measure the current angle of the joint. The home position for all joints is also depicted in the figure above.
+ press **r** to enter "set maximum angle" mode.
+ use **w**,**s** to set the angle.
+ repeat the steps for the minimum angle in the same way.
+ one max/min angle and PWM value are set for one joint, press **q** to move to home (0°) position. If the robot
does not move to 0° according to the above diagram, adjust the angles or PWM values accordingly.
+ when done, press **p** to print the configuration. Copy and paste it into the `config.h` file.

```
 INCREMENT 'w'
 DECREMENT 's'
 CHANGE_MIN_ANGLE 'e'
 CHANGE_MAX_ANGLE 'r'
 CHANGE_MIN_FREQUENCY 't'
 CHANGE_MAX_FREQUENCY 'z'
 MOVE_TO_ZERO 'q'
 SET_MOVE 'n'
 PRINT_CONFIG 'p'
 CHANGE_MODE 'm'
 PRINT_HELP 'h'
```

## Kinematics Logical - Physical Angles
The MRC uses two types of angles - logical and physical angles.
Logical joint angles are used internally by the robot controller and are the result of the IK calculation. The robot is therefore modeled as a series of joints and links. However, a real physical robot may use mechanical links to move some axis. Therefore moving R1 may also move R2, because of the kinematic linking.

To account for mechanical kinematics, the methods logicalToPhysicalAngles and physicalToLogicalAngles are used.
On the robot depicted below, both motors for J1 and J2 are at the position of J1. A link is used to drive J2.
Therefore moving only R1 also decreases the angle of J2. To not move J2, R2 has to move the same amount as R1.

<img width="1151" alt="bildschirmfoto 2017-05-15 um 12 13 07" src="https://cloud.githubusercontent.com/assets/3062564/26053118/e25bf600-3967-11e7-90ac-8fb8fd248fc0.png">

This is represented using the methods like this:

```cpp
void logicalToPhysicalAngles(float angles[6]) {
    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];
}
```

## Tests
For testing and mocking gMock/gTest is used.
To run the tests `/tests/test.h`.

## Debug
increase the log-level in `Logger.cpp` to see whats happening in the controller.

## Troubleshooting

**MRC does not respond** unexprected data in the EEPROM may freeze the MRC. In main.cpp uncomment `// Eepromstorage.clear();` in line 75.

**can not send commands** make sure your terminal monitor sends a '\r' when pressing return. Otherwise in MRCP.h change the MRCP `MRCP_END_FRAME = '\r'` to any unused char.

## todo
+ implement circular interpolation methods
+ implement proper logging
+ dependency inject the logger
+ implement interpolation of poses (quaternion slerp?)
