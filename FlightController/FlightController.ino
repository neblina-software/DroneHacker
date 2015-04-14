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
* - Evitar lo mas posible los comentarios en el codigo
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
#include "I2Cdev.h"
#include <PID_v1.h>

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/**************
*    SIGNAL   *
*    CHECKER  *
**************/

int power = 0;

/****************
* STABILIZER AT *
****************/

#define MPU_STABILIZER_ACTIVATION 1

/**************
*     RX/TX   *
*     PINS    *
**************/

#define THROTTLE_IN_PIN 3
#define PITCH_IN_PIN 4

/**************
*     SERVO   *
*     PINS    *
**************/

#define MOTORTL_OUT_PIN 8
#define MOTORTR_OUT_PIN 9
#define MOTORBR_OUT_PIN 11
#define MOTORBL_OUT_PIN 13

Servo servoMotorTL;
Servo servoMotorTR;
Servo servoMotorBR;
Servo servoMotorBL;

/*************************
*          PID           *
*       CALIBRATION      *
* kp = Proportional term *
* ki = Integral term     *
* kd = Derivative term   *
*************************/

int kp = 7;
int ki = 2;
int kd = 1;

double pitchSetpoint, pitchInput, pitchOutput;
double rollSetpoint, rollInput, rollOutput;

/**************
*   X: Pitch  *
*   Y: Roll   *
*   Z: Yaw    *
**************/

PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, kp, ki, kd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, kp, ki, kd, DIRECT);

/**************
*     MPU     *
**************/

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

int outputTL, outputTR, outputBR, outputBL, auxTL, auxTR, auxBR, auxBL;
int mpuYaw, mpuPitch, mpuRoll;

/**************
*    RX/TX    *
**************/

#define THROTTLE_FLAG 1
#define PITCH_FLAG 1

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unThrottleInShared;
volatile uint16_t unPitchInShared;

uint32_t ulThrottleStart;
uint32_t ulPitchStart;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
  
  /*****************
  *       PID      *
  * INITIALIZATION *
  ******************/
  //Input = 0;
  //Setpoint = 0;
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(0, 200);
  rollPID.SetOutputLimits(0, 200);
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24;
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  Serial.begin(115200);
  Serial.println("multiChannels");

  servoMotorTL.attach(MOTORTL_OUT_PIN);
  servoMotorTR.attach(MOTORTR_OUT_PIN);
  servoMotorBL.attach(MOTORBL_OUT_PIN);
  servoMotorBR.attach(MOTORBR_OUT_PIN);

  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE); 
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch, CHANGE);
  
  arm();
  
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  
  if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
    
  static uint16_t unThrottleIn;
  static uint16_t unPitchIn;

  static uint8_t bUpdateFlags;

  if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
    }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  mpuYaw = ypr[0] * 180/M_PI;
  mpuPitch = ypr[1] * 180/M_PI;
  mpuRoll = ypr[2] * 180/M_PI;

  if(bUpdateFlagsShared) {
    
    power = 0;
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;
    
    if(bUpdateFlags & THROTTLE_FLAG) {
      unThrottleIn = unThrottleInShared;
    }
    if(bUpdateFlags & PITCH_FLAG) {
      unPitchIn = unPitchInShared;
    }
    
    bUpdateFlagsShared = 0;
    interrupts();
    
  }
   
  if(!bUpdateFlags) {
    // Iniciar contador
    power++;
  }
  
  if(power > 2) {
    Serial.println("No Signal!");
    outputTL = 1000;
    outputTR = 1000;
    outputBL = 1000;
    outputBR = 1000;
    delay(500);
  }
  
  if(bUpdateFlags & THROTTLE_FLAG) {
    if(servoMotorTL.readMicroseconds() && servoMotorTR.readMicroseconds() 
        && servoMotorBL.readMicroseconds() && servoMotorBR.readMicroseconds()
        != unThrottleIn) {
          if(unThrottleIn > 1745) {
            outputTR = 1745;
            outputTL = 1745;
            outputBL = 1745;
            outputBR = 1745;
            auxTR = 1745;
            auxTL = 1745;
            auxBL = 1745;
            auxBR = 1745;
          }
          outputTR = unThrottleIn;
          outputTL = unThrottleIn;
          outputBL = unThrottleIn;
          outputBR = unThrottleIn;
          auxTR = unThrottleIn;
          auxTL = unThrottleIn;
          auxBL = unThrottleIn;
          auxBR = unThrottleIn;
          /*************
          * Stabilizer *
          **************/
          if(unThrottleIn > 1060) {
            pitchInput = MPU_STABILIZER_ACTIVATION; // Not less than 3 or not equals to 0
            pitchSetpoint = abs(mpuPitch);
            pitchPID.Compute();
            rollInput = MPU_STABILIZER_ACTIVATION; // Not less than 3 or not equals to 0
            rollSetpoint = abs(mpuRoll);
            rollPID.Compute();
            if(mpuPitch > MPU_STABILIZER_ACTIVATION) {
               outputTL = unThrottleIn + pitchOutput;
               outputBL = unThrottleIn + pitchOutput;
            }
            if(mpuPitch < -MPU_STABILIZER_ACTIVATION) {
               outputTR = unThrottleIn + pitchOutput;
               outputBR = unThrottleIn + pitchOutput;
            }
            if(mpuRoll > MPU_STABILIZER_ACTIVATION) {
               outputBL = unThrottleIn + rollOutput;
               outputBR = unThrottleIn + rollOutput;
            }
            if(mpuRoll < -MPU_STABILIZER_ACTIVATION) {
               outputTL = unThrottleIn + rollOutput;
               outputTR = unThrottleIn + rollOutput;
            }
          }
    }
  }
  
  if(bUpdateFlags & PITCH_FLAG) {
    if(servoMotorTL.readMicroseconds() && servoMotorTR.readMicroseconds() 
        && servoMotorBL.readMicroseconds() && servoMotorBR.readMicroseconds()
        != unPitchIn) {
          //Serial.println(unPitchIn);
          if(unPitchIn > 1550) {
            pitchSetpoint = map(unPitchIn, 1550, 2000, 0, 30);
            pitchPID.Compute();
            outputTL = auxTL + pitchOutput;
            outputBL = auxBL + pitchOutput;
          }
          if(unPitchIn < 1450) {
            pitchSetpoint = map(unPitchIn, 1000, 1450, 0, 30);
            pitchPID.Compute();
            outputTR = auxTR + pitchOutput;
            outputBR = auxBR + pitchOutput;
          }
          /**
          if(unPitchIn > 1450 || unPitchIn < 1550) {
              pitchSetpoint = 0;
              pitchPID.Compute();
          }
          **/
    }
  }
  
  initMotors(
            outputTL,
            outputTR,
            outputBR,
            outputBL
  );
  
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

void calcPitch() {
  if(digitalRead(PITCH_IN_PIN) == HIGH) { 
    ulPitchStart = micros();
  } else{
    unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}

void arm() {
  servoMotorTL.writeMicroseconds(1000);
  servoMotorTR.writeMicroseconds(1000);
  servoMotorBL.writeMicroseconds(1000);
  servoMotorBR.writeMicroseconds(1000);
}

void initMotors(int tl, int tr, int br, int bl) {
    
  Serial.println(tl);
  Serial.println(tr);
  Serial.println(br);
  Serial.println(bl);
  
  servoMotorTL.writeMicroseconds(tl);
  servoMotorTR.writeMicroseconds(tr);
  servoMotorBR.writeMicroseconds(br);
  servoMotorBL.writeMicroseconds(bl);
  
}
