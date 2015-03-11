/**
*  DroneHacker (c) 2015 Anibal Gomez
*  TX / RX
*  Ref: https://www.sparkfun.com/tutorials/348
**/

/*
 RC PulseIn Joystick
 By: Nick Poole
 SparkFun Electronics
 Date: 5
 License: CC-BY SA 3.0 - Creative commons share-alike 3.0
 use this code however you'd like, just keep this license and
 attribute. Let me know if you make hugely, awesome, great changes.
 */

int ch1; // Here's where we'll keep our channel values
int ch2;
int ch3;

void setup() {

pinMode(5, INPUT); // Set our input pins as such
pinMode(6, INPUT);
pinMode(7, INPUT);

Serial.begin(9600); // Pour a bowl of Serial

}

void loop() {
  
  ch1 = pulseIn(5, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(6, HIGH, 25000); // each channel
  ch3 = pulseIn(7, HIGH, 25000);
  
  if(ch1>1000){Serial.println("Left Switch: Engaged");} 
  if(ch1<1000){Serial.println("Left Switch: Disengaged");}
  /* I found that Ch1 was my left switch and that it 
  floats around 900 in the off position and jumps to 
  around 1100 in the on position */

Serial.print("Right Stick X:"); // Ch3 was x-axis 
Serial.println(map(ch3, 1000,2000,-500,500)); // center at 0

Serial.print("Right Stick Y:"); // Ch2 was y-axis
Serial.println(map(ch2, 1000,2000,-500,500)); // center at 0

Serial.println(); //make some room

delay(100);// I put this here just to make the terminal 
           // window happier
}
