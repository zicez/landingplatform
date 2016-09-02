#include <ros.h>
#include <std_msgs/Empty.h>
#include "BasicStepperDriver.h"

ros::NodeHandle nh;

//Set up variables for the stepper motor
#define DIR 8
#define STEP 9
#define SLEEP 10
#define ENABLE 11
#define HOME 12
#define MICROSTEPS 1
#define MOTOR_STEPS 200
#define moveLength 2950 // The length of mvoement. Different per type of drone.
int currentStatus = 1; // 1 = not enclosed. -1 = enclosed

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE, SLEEP, HOME);

//Open/close the guide
void toggle()
{
  stepper.wakeup();
  stepper.move(moveLength * currentStatus);
  currentStatus = -1 * currentStatus;
  stepper.sleep();
}

//Initialize a subscriber
ros::Subscriber<std_msgs::Empty> sub("toggle", &toggle);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
