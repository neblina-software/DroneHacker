/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup()
{
  Serial.begin(115200);
  //initialize the variables we're linked to
  Input = 0;
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(1000, 2000);
}

void loop()
{
  Input = 80;
  myPID.Compute();
  Serial.println(Output);
}


