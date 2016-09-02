#include <Arduino.h>
#include "BasicStepperDriver.h"

//Set up variables for the stepper motor
#define HOME 12
#define SLEEP 10
#define ENABLE 11
#define DIR 8 
#define STEP 9
#define MOTOR_STEPS 200
#define moveLength 2950 // The length of mvoement. Different per type of drone. 
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE, SLEEP, HOME);

unsigned long serialdata;
int inbyte;
int stepNumber;
//Step length = 2950
void setup()
{
  stepper.setRPM(200);
  Serial.begin(9600);
}
  /*stepper.wakeup();
  stepper.move(1000);
  delay(500);
  stepper.move(-1100);
  stepper.sleep();
  delay(500);*/
  
void loop()
{
  getSerial();
  switch(serialdata)
  {
  case 1:
    {
      //analog digital write
      getSerial();
      switch (serialdata)
      {
      case 1:
        {
          //Forward
          getSerial();
          stepNumber = serialdata;
          stepper.wakeup();
          stepper.move(stepNumber);
          stepper.sleep();
          Serial.println(stepNumber);
          stepNumber = 0;
          break;
        }
      case 2:
        {
          //Backward
          getSerial();
          stepNumber = serialdata* -1;
          stepper.wakeup();
          stepper.move(stepNumber);
          stepper.sleep();
          Serial.println(stepNumber);
          stepNumber = 0;
          break;
          
        }
     }
     break; 
   }
  }
}

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
     
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}
