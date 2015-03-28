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

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


int debugConsole = 0;

Servo motorNW;
Servo motorNE;
Servo motorSE;
Servo motorSW;

// See Mode 2 Diagram
int ch1Aileron; // Roll
int ch2Elevator; // Pitch
int ch3Throttle; // Throttle
int ch4Rudder; // Yaw
int ch5LandingGear; // Flight Mode

int vlevitate;
int vmove;

int armed = 0;
int power = 0;

int armCounting = 0;
int disarmCounting = 0;

int motorSpeed;

#define NE_Min 54
#define NE_Max
#define SW_Min 43
#define SW_Max 133
#define SE_Min 43
#define SE_Max 133

// A2212-13T
int calibratedMin = 0;
int calibratedMax = 180;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  
  // --- mpu -----
   // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  // ---- mpu -----
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
  
  Serial.begin(115200);
  Serial.println("iniciando...");

}

void loop() {
  
  // -- mpu --
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

    }
  // -- mpu --
  
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
    Serial.println(map(ch1Aileron, 1000, 2000, calibratedMin, calibratedMax));    
    Serial.print("Canal 2 Elevator:");
    Serial.println(map(ch2Elevator, 1000, 2000, calibratedMin, calibratedMax));
    Serial.print("Canal 3 Throttle:");
    Serial.println(map(ch3Throttle, 1000, 2000, calibratedMin, calibratedMax));
    Serial.print("Canal 4 Rudder:");
    Serial.println(map(ch4Rudder, 1000, 2000, calibratedMin, calibratedMax));
    Serial.print("Canal 5 Landing:");
    Serial.println(map(ch5LandingGear, 1000, 2000, calibratedMin, calibratedMax));
    Serial.println();
    delay(500);
  }
  
  vlevitate = map(ch3Throttle, 1000, 2000, calibratedMin, calibratedMax); // rx - tx values -> pwm values
  vlevitate = constrain(vlevitate , calibratedMin, calibratedMax);
  
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
        stabilized(motorSpeed);
      }
      
    }
             
}

void stabilized(int motorSpeed) {
  motorNW.write(motorSpeed);
  motorNE.write(motorSpeed);
  motorSE.write(motorSpeed);
  motorSW.write(motorSpeed);
}

