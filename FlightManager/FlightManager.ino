/*
* DroneHacker (c) 2015 Anibal Gomez
* ---------------------------------
* Software Controlador De Vuelo
* ---------------------------------
*
*            NW      NE
*            |\     /|
*            ( \   / )
*             \ (_) / 
*              ) _ (  
*             / ( ) \ 
*            ( /   \ )
*            |/     \|
*            SW     SE
*       
* ---------------------------------
* anibalgomez@icloud.com
* ---------------------------------
*
* LICENSE
*
* This source file is subject to the new BSD license that is bundled
* with this package in the file LICENSE.txt.
*
**/

#include <Servo.h>

int debugConsole = 1;

Servo motorNW;
Servo motorNE;
Servo motorSE;
Servo motorSW;

int ch1Aileron;
int ch2Elevator;
int ch3Throttle;
int ch4Rudder;
int ch5LandingGear;

int vlevitate;
int vmove;

int armed = 0;

int startCounting = 0;

void setup() {
  
  motorNW.attach(8);
  motorNE.attach(9);
  motorSE.attach(11);
  motorSW.attach(13);
  
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  
  stabilized(0);
  delay(30);
  stabilized(30);
  
  Serial.begin(9600);
  Serial.println("iniciando...");

}

void loop() {
  
  startedTime = millis();
 
  // Read the pulse width of each channel
  ch1Aileron = pulseIn(3, HIGH, 25000);
  ch2Elevator = pulseIn(4, HIGH, 25000);
  ch3Throttle = pulseIn(5, HIGH, 25000);
  ch4Rudder = pulseIn(6, HIGH, 25000);
  ch5LandingGear = pulseIn(7, HIGH, 25000);
  
  if(ch1Aileron > 1000) Serial.println("Transmisor encendido"); 
  if(ch1Aileron < 1000) Serial.println("Transmisor apagado");

  arm();

  if(debugConsole == 1) {                                 
    Serial.print("Left Stick Y:");
    Serial.println(map(ch3Throttle, 1000, 2000, -500, 500));
    Serial.print("Right Stick Y:");
    Serial.println(map(ch2Elevator, 1000, 2000, -500, 500));
    Serial.println();
    delay(100);
  }
  
  vlevitate = map(ch3Throttle, 1000, 2000, -500, 500); // rx - tx values
  vlevitate = constrain(vlevitate, 40, 180); // valid pwm values
  
  //qmove = map(ch2Elevator, 1000,2000, -500, 500);
  //qmove = constrain(qmove, -255, 255);
      
  if(vlevitate > 53){
    stabilized(vlevitate);
  }
  if(vlevitate < 53) {
    stabilized(vlevitate);
  }
             
}

void arm() {
    
  initAileron =  map(ch1Aileron, 1000, 2000, -500, 500);
  initThrottle = map(ch3Throttle, 1000, 2000, -500, 500);
  if(initThrottle < -480 && initAileron > 480) {
    startCounting = startCounting + 1000;
  } else {
    startCounting = 0;
  }
  
  if(startCounting == 30000) {
    Serial.println("Vehiculo Armado...");
    armed = 1;
  }
  
}

void stabilized(int motorSpeed) {
  if(armed == 1) {
    motorNW.write(motorSpeed);
    motorNE.write(motorSpeed);
    motorSE.write(motorSpeed);
    motorSW.write(motorSpeed);
  }
}
