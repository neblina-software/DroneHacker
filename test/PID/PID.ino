/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

int kp = 2;
int ki = 5;
int kd = 1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

void setup()
{
  Serial.begin(115200);
  //initialize the variables we're linked to
  Input = 0;
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 170);
}

void loop()
{
  Input = 0;
  Setpoint = 30;
  myPID.Compute();
  Serial.println(Output);
}


