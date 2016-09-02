#include <Arduino.h>
#include "BasicStepperDriver.h"

//Set up variables for the stepper motor
#define HOME 4
#define SLEEP 10
#define ENABLE 11
#define DIR 8 
#define STEP 9
#define MOTOR_STEPS 200
#define moveLength 2950 // The length of mvoement. Different per type of drone. 
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE, SLEEP, HOME);

void setup()
{
  stepper.setRPM(200);
}
  
void loop()
{
  stepper.wakeup();
  stepper.move(200);
  stepper.move(-200);
  stepper.sleep();
}
