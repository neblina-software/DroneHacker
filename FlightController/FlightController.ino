/*
* DroneHacker (c) 2015 Anibal Gomez
* ---------------------------------
* Software Controlador De Vuelo
* ---------------------------------
*
* Quadcoptero
* -----------
*
*             - Eje Y -
*
*                RX
*            TL      TR
*            |\     /|
*            ( \   / )
*             \ (_) / 
*              ) _ (   - Eje X -
*             / ( ) \ 
*            ( /   \ )
*            |/     \|
*            BL     BR
*       
*
* CONTRIBUCIONES
* ==============
* 
* - Si desea contribuir con el programa, por favor:
* - No comente ninguna linea de codigo
* - Envie su documentacion externa, en la carpeta, /docs/
* - Use estandares de convencion
* - Use variables lo mas descriptivas posibles
* - Use DocBlocks en la parte superior de cada metodo
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

#include <PinChangeInt.h>
#include <Servo.h>

#define THROTTLE_IN_PIN 5

#define MOTORTL_OUT_PIN 8
#define MOTORTR_OUT_PIN 9
#define MOTORBR_OUT_PIN 11
#define MOTORBL_OUT_PIN 13

Servo servoMotorTL;
Servo servoMotorTR;
Servo servoMotorBR;
Servo servoMotorBL;

#define THROTTLE_FLAG 1

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unThrottleInShared;

uint32_t ulThrottleStart;

void setup(){
  
  Serial.begin(115200);
  Serial.println("multiChannels");

  servoMotorTL.attach(MOTORTL_OUT_PIN);
  servoMotorTR.attach(MOTORTR_OUT_PIN);
  servoMotorBL.attach(MOTORBL_OUT_PIN);
  servoMotorBR.attach(MOTORBR_OUT_PIN);

  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE); 
  
  arm();

}

void loop() {
  
  static uint16_t unThrottleIn;

  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared) {
    
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;
    if(bUpdateFlags & THROTTLE_FLAG) {
      unThrottleIn = unThrottleInShared;
    }
    bUpdateFlagsShared = 0;
    interrupts();
    
  }
  
  if(bUpdateFlags & THROTTLE_FLAG) {
    if(servoMotorTL.readMicroseconds() && servoMotorTR.readMicroseconds() 
        && servoMotorBL.readMicroseconds() && servoMotorBR.readMicroseconds()
        != unThrottleIn) {
          Serial.println(unThrottleIn);
          stabilized(unThrottleIn);
    }
  }
  
  bUpdateFlags = 0;
}

void calcThrottle() {
  if(digitalRead(THROTTLE_IN_PIN) == HIGH) { 
    ulThrottleStart = micros();
  } else{
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void arm() {
  servoMotorTL.writeMicroseconds(1000);
  servoMotorTR.writeMicroseconds(1000);
  servoMotorBL.writeMicroseconds(1000);
  servoMotorBR.writeMicroseconds(1000);
}

void stabilized(uint16_t unThrottleIn) {
  servoMotorTL.writeMicroseconds(unThrottleIn);
  servoMotorTR.writeMicroseconds(unThrottleIn);
  servoMotorBL.writeMicroseconds(unThrottleIn);
  servoMotorBR.writeMicroseconds(unThrottleIn);
}

