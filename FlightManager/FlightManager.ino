/*
* DroneHacker (c) 2015 Anibal Gomez
* ---------------------------------
* Software Controlador De Vuelo
* ---------------------------------
*
* Quadcoptero
* -----------
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

int debugConsole = 0;

Servo motorNW;
Servo motorNE;
Servo motorSE;
Servo motorSW;

int ch1Aileron; // Flight Mode
int ch2Elevator;
int ch3Throttle;
int ch4Rudder;
int ch5LandingGear; // Roll

int vlevitate;
int vmove;

int armed = 0;
int power = 0;

int armCounting = 0;
int disarmCounting = 0;

int motorSpeed;

// A2212-13T
int calibrateMin = 43;
int calibrateMax = 180;

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
  
  // arm
  motorNW.write(0);
  motorNE.write(0);
  motorSE.write(0);
  motorSW.write(0);
  delay(30);
  motorNW.write(30);
  motorNE.write(30);
  motorSE.write(30);
  motorSW.write(30);
  
  Serial.begin(9600);
  Serial.println("iniciando...");

}

void loop() {
  
  /**
  if(armed == 1) {
    Serial.println("El vehiculo se encuentra armando... (Precaucion)");
  } else {
    Serial.println("Vehiculo desarmado...");
  }
  **/
  
  //startedTime = millis();
 
  // Read the pulse width of each channel
  ch1Aileron = pulseIn(3, HIGH, 25000);
  ch2Elevator = pulseIn(4, HIGH, 25000);
  ch3Throttle = pulseIn(5, HIGH, 25000);
  ch4Rudder = pulseIn(6, HIGH, 25000);
  ch5LandingGear = pulseIn(7, HIGH, 25000);
  
  if(ch1Aileron > 1) {
    Serial.println("Transmisor encendido");
    power = 1;
  }
  
  if(ch1Aileron < 1) {
    Serial.println("Transmisor apagado");
    power = 0;
  }

/**
  if(power == 1) {
    if(ch3Throttle < -440 && ch5LandingGear < -440) {
      Serial.println("Empezando conteo: ");
      armCounting = armCounting + 1000;
      Serial.println(armCounting);
    } else {
      armCounting = 0;
    }
    if(armCounting == 15000) {
      Serial.println("Vehiculo Armado...");
      armCounting = 0;
      armed = 1;
    }
  }
  
  if(ch3Throttle < -440 && ch5LandingGear > 0) {
    Serial.println("Desarmando...: ");
    disarmCounting = disarmCounting + 1000;
    Serial.println(disarmCounting);
  }
  if(disarmCounting == 10000) {
    Serial.println("Vehiculo Desarmado...");
    disarmCounting = 0;
    armed = 0;
  }
  **/

  if(debugConsole == 1) {   
    Serial.print("Canal 1 Aileron:");
    Serial.println(map(ch1Aileron, 1000, 2000, -500, 500));    
    Serial.print("Canal 2 Elevator:");
    Serial.println(map(ch2Elevator, 1000, 2000, -500, 500));
    Serial.print("Canal 3 Throttle:");
    Serial.println(map(ch3Throttle, 1000, 2000, -500, 500));
    Serial.print("Canal 4 Rudder:");
    Serial.println(map(ch4Rudder, 1000, 2000, -500, 500));
    Serial.print("Canal 5 Landing:");
    Serial.println(map(ch5LandingGear, 1000, 2000, -500, 500));
    Serial.println();
    delay(500);
  }
  
  vlevitate = map(ch3Throttle, 1000, 2000, 0, 180); // rx - tx values -> pwm values
  vlevitate = constrain(vlevitate , 0, 180);
  
  Serial.print("Pasar valor a motor -> ");
  Serial.println(vlevitate);
  
  //qmove = map(ch2Elevator, 1000,2000, -500, 500);
  //qmove = constrain(qmove, -255, 255);
      
    if(power == 1) {
      if(vlevitate <= 30) {
        motorSpeed = 40;
      }else if(vlevitate <= 40) {
        motorSpeed = 44;
      } else {
        motorSpeed = vlevitate;
      }
      
      Serial.print("Arrancando motores a: ");
      Serial.println(motorSpeed);
      
      if(debugConsole != 1) {
        motorNW.write(motorSpeed);
        motorNE.write(motorSpeed);
        motorSE.write(motorSpeed);
        motorSW.write(motorSpeed);
      }
      
    }
             
}

