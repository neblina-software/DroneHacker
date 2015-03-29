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

#include "I2Cdev.h"
#include <Servo.h>

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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

Servo motorTL, motorTR, motorBL, motorBR;

int armed = 0;
int power = 0;

int motorSpeed;
int count;

// A2212-13T
int calibratedMin = 0;
int calibratedMax = 180;

long aileron; // Roll
long elevator; // Pitch
long throttle; // Throttle
long rudder; // Yaw
long landingGear; // Flight Mode

int yaw, pitch, roll;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
  // Motors
  motorTL.attach(8);
  motorTR.attach(9);
  motorBR.attach(11);
  motorBL.attach(13);
  
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  
  // Arm
  motorTL.write(0);
  motorTR.write(0);
  motorBL.write(0);
  motorBR.write(0);
  delay(30);
  motorTL.write(30);
  motorTR.write(30);
  motorBL.write(30);
  motorBR.write(30);
  Serial.println("Armando vehiculo...");
  delay(20);

}

void loop() {
     
  aileron = readChannel(3); // Roll
  elevator = readChannel(4); // Pitch
  throttle = readChannel(5); // Throttle
  rudder = readChannel(6); // Yaw
  landingGear = readChannel(7); // Flight Mode
  
    if(aileron == -180 && elevator == -180 && throttle == -180) {
      power = 0;
    } else {
      power = 1;
    }
     
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
    
    if(power == 1) {
      if(throttle > -180) {
        motorSpeed = 53;
      } else {
        motorSpeed = 43;
      }
    }
    
     stabilized(motorSpeed);
  
}

void stabilized(int motorSpeed) {
  int tl, tr, bl, br;
  
  tl = motorSpeed;
  tr = motorSpeed;
  bl = motorSpeed;
  br = motorSpeed;
        
  #ifdef OUTPUT_READABLE_CHANNELS
  Serial.print("Speed: ");
  Serial.println(motorSpeed);
  #endif
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  yaw = ypr[0] * 180/M_PI;
  pitch = ypr[1] * 180/M_PI;
  roll = ypr[2] * 180/M_PI;
  
  if(pitch > 3) {
    tl = motorSpeed + 20;
    bl = motorSpeed + 20;
  } else {
    tl = motorSpeed;
    bl = motorSpeed;
  }
  
  if(pitch < -3) {
    tr = motorSpeed + 20;
    br = motorSpeed + 20;
  } else {
    tr = motorSpeed;
    br = motorSpeed;
  }
  
  if(roll > 3) {
    bl = motorSpeed + 20;
    br = motorSpeed + 20;
  } else {
    bl = motorSpeed;
    br = motorSpeed;
  }
  
  if(roll < 3) {
    tl = motorSpeed + 20;
    tr = motorSpeed + 20;
  } else {
    tl = motorSpeed;
    tr = motorSpeed;
  }
  
  motorTL.write(tl);
  motorTR.write(tr);
  motorBL.write(bl);
  motorBR.write(br);
  
}

/**
* Read the pulse width of each channel
* RX / TX -> PWM
* @author Anibal Gomez <anibalgomez@icloud.com>
**/
long readChannel(int pin) {
  long ch = pulseIn(pin, HIGH, 25000);
  long mapValue = map(ch, 1000, 2000, -500, 500);
  return constrain(mapValue, -180, 180);
}

