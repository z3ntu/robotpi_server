# RobotPi

RobotPi-Server is a server application to control a robot. You can control it via the [Android App](https://github.com/z3ntu/robotpi_client_android) or directly via telnet (port 2048).
It is designed for the Raspberry Pi.

### Prerequisites
For compiling RobotPi-Server you need:
- [wiringPi](http://wiringpi.com/)
- [BCM2835](http://www.airspayce.com/mikem/bcm2835/)

### How to compile
```
make
```

### How to run
```
./robotpi_socket
```

### Communication
Communication happens on port 2048 (configurable). All messages (to the server) contain four characters.

**Servo**: "Y" and then a three-digit number between 0 & 140.

**Motors**: two characters identifier what to do and then a two-digit number between 0 & 100.

*Identifiers*:
- Forward left: **FL**
- Forward right: **FR**
- Backwards left: **BL**
- Backwards right: **BR**
- Forward: **F0**
- Backwards: **B0**
- Rotate left: **RL**
- Rotate right: **RR**

*Extra commands*:
- Reset all:        **0000**
- Disconnect:       **C001**
- Auto:             **AUTO** (currently not working great)
- Test:             **TEST** (do some dancing :dancers: ; nope just some turning and stuff)
- Distance:         **DIST** (kinda working)

### Photos
![Photo 1](http://z3ntu.github.io/images/robotpi_1.jpg)
![Photo 2](http://z3ntu.github.io/images/robotpi_2.jpg)
![Photo 3](http://z3ntu.github.io/images/robotpi_3.jpg)

### Connections
A H-Bridge is connected with the physical pins 35,37 & 38,40.
The servo is connected on physical pin 12.
