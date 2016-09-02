StepperDriver
=============

Adapted from <a href="https://github.com/laurb9/StepperDriver">laurb9/StepperDriver</a>  
Motors
======

4-wire bipolar stepper motor or some 6-wire unipolar in 4-wire configuration (leaving centers out).

Connections
===========

Minimal configuration from <a href="https://www.pololu.com/product/2134">Pololu DRV8834 page</a>:

<img src="https://a.pololu-files.com/picture/0J4344.600.png">

Wiring
======

- Arduino to driver board:
    - DIR - PIN 8
    - STEP - PIN 9
    - SLEEP - PIN 10
    - DISABLE - PIN 11
    - HOME - PIN 12
    - GND - Arduino GND
    - GND - Motor power GND
    - VMOT - Motor

- 100uF capacitor between GND - VMOT
- Set the max current on the driver board to the motor limit (see below).
- Have a motor power supply that can deliver that current.

Set Max Current
===============

The max current is set via the potentiometer on board.
Turn it while measuring voltage at the passthrough next to it.
The formula is V = I*5*R where I=max current, R=current sense resistor installed onboard

- DRV8834 Pololu board, R=0.1 and V = 0.5 * max current(A).
  For example, for 1A you will set it to 0.5V.

- DRV8825 low-current board, R=0.33 and V = 1.65 * max current(A).
  For example, for 0.5A the reference voltage should be 0.82V

Hardware
========
- Arduino-compatible board
- A <a href="https://www.pololu.com/category/120/stepper-motor-drivers">stepper motor driver</a>
- A <a href="http://www.circuitspecialists.com/stepper-motor">Stepper Motor</a>.
- 1 x 100uF capacitor or higher

Operations
==========
The Arduino subscribes to the topic "toggle" and waits for the signal to trigger the stepper motors. Each trigger either moves it back or forward.

ROS Subscriber
==============

```
ros::Subscriber<std_msgs::Empty> sub("toggle", &toggle);

void toggle()
{
  stepper.wakeup();
  stepper.move(moveLength * currentStatus);
  currentStatus = -1 * currentStatus;
  stepper.sleep();
}
```
