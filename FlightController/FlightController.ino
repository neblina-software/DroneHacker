/*
* DroneHacker (c) 2016 Anibal Gomez
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
* NOTAS
* =====
* 
* - Dado que el español es mi lenguaje materno, los comentarios del código serán
*   en éste lenguaje, pero los nombres de variables y funciones seran en lenguaje inglés,
*   puesto que es más fácil mantener la mente concentrada en un sólo lenguaje.
* 
* MEJORAS
* =======
* 
* 07/02/2016: Para la gente que probó el código antes de ésta fecha:
* 
* + El código tenia un error y los valores estaban invertidos y se ha mejorado.
*   + Setpoint: Corresponde al valor de entrada (MPU).
*   + Input: Corresponde al valor de entrada del radio control.
*   
* Si deseas saber más investiga un poco más acerca de "Controladores P.I.D.".
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

int outputTL, outputTR, outputBR, outputBL, auxTL, auxTR, auxBR, auxBL;
int mpuYaw, mpuPitch, mpuRoll;

/***************************
 * Ganancia De PID a Motor *
 * Mapeo de 1000 a 2000    *
 * (Se cuidadoso aqui)     *
 **************************/
int motorGain = 70;

/**************
*   CHECADOR  *
*    SEÑAL    *
**************/

int power = 0;

/*******************************************
* VALOR DONDE SE ACTIVARA EL ESTABILIZADOR *
********************************************/

#define MPU_STABILIZER_ACTIVATION 2

/******************
*   RADIO CONTROL *
*     RX/TX       *
*     PINS        *
*******************/

#define THROTTLE_IN_PIN 3 // Gas
#define PITCH_IN_PIN 4 // Elevator
#define ROLL_IN_PIN 5 // Aileron

/********************************
*  Electronic Speed Controllers *
*            SERVO              *
*            PINS               *
*********************************/

#define MOTORTL_OUT_PIN 8
#define MOTORTR_OUT_PIN 9
#define MOTORBR_OUT_PIN 11
#define MOTORBL_OUT_PIN 13

Servo servoMotorTL;
Servo servoMotorTR;
Servo servoMotorBR;
Servo servoMotorBL;

/********************************
*          PID                  *
*       CALIBRACION             *
* kp = Proporcional             *
* ki = Integrativo              *
* kd = Derivativo               *
*********************************
* Prefijo agg = agresivo        *
* Prefijo cons = conservativo   *
*********************************/

//float kp = .20;
//float ki = .040;
//float kd = .100;

// Tuning Parametros
// http://diydrones.com/page/pid-tuning-demos
double PitchaggKp=.40, PitchaggKi=0.02, PitchaggKd=.9;
double PitchconsKp=.53, PitchconsKi=0.02, PitchconsKd=0.12;

double RollaggKp=.40, RollaggKi=0.02, RollaggKd=.9;
double RollconsKp=.53, RollconsKi=0.02, RollconsKd=0.12;


/*************************
*          PID           *
*         LIMITES        *
*************************/

#define OUTPUT_LIMITS 30

double pitchSetpoint, pitchInput, pitchOutput;
double rollSetpoint, rollInput, rollOutput;

/**************
*   X: Pitch  *
*   Y: Roll   *
*   Z: Yaw    *
**************/

//Specify the links and initial tuning parameters
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PitchconsKp, PitchconsKi, PitchconsKd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, RollconsKp, RollconsKi, RollconsKd, DIRECT);

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

/**************
*    RX/TX    *
**************/

#define THROTTLE_FLAG 1
#define PITCH_FLAG 1
#define ROLL_FLAG 1

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unThrottleInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

uint32_t ulThrottleStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
  
  /*****************
  *       PID      *
  * INICIALIZACION *
  ******************/
  pitchInput = 0;
  rollInput = 0;

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(0, OUTPUT_LIMITS);
  rollPID.SetOutputLimits(0, OUTPUT_LIMITS);
  
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
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll, CHANGE);
  
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
  static uint16_t unRollIn;

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
    if(bUpdateFlags & ROLL_FLAG) {
      unRollIn = unRollInShared;
    }
    
    bUpdateFlagsShared = 0;
    interrupts();
    
  }
   
  if(!bUpdateFlags) {
    // Iniciar contador
    power++;
  }

  /**
  if(power > 2) {
    Serial.println("No Signal!");
    outputTL = 1000;
    outputTR = 1000;
    outputBL = 1000;
    outputBR = 1000;
    delay(500);
  }
  **/

  pitchSetpoint = mpuPitch; // Valor Deseado (en grados)
  pitchInput = map(unPitchIn, 900, 2000, -30, 30); // Valor de entrada (necesita convertirse a grados)
  double Pitchgap = abs(pitchSetpoint-pitchInput); // Distancia hasta setpoint (Error)
  if(Pitchgap<5) {  // Estamos lejos del setpoint, usar parametros conservativos
      pitchPID.SetTunings(PitchconsKp, PitchconsKi, PitchconsKd);
  } else {
    // Estamos muy cerca del setpoint, usa parametros agresivos
    pitchPID.SetTunings(PitchaggKp, PitchaggKi, PitchaggKd);
  }
  pitchPID.Compute();

  rollSetpoint = mpuRoll; // Valor deseado (necesita convertirse a grados)
  rollInput = map(unRollIn, 900, 2000, -30, 30); // Valor de entrada (necesita convertirse a grados)
  double Rollgap = abs(rollSetpoint-rollInput); // Distancia hasta setpoint (Error)
  if(Rollgap<5) {  // Estamos lejos del setpoint, usar parametros conservativos
      rollPID.SetTunings(RollconsKp, RollconsKi, RollconsKd);
  } else {
    // Estamos muy cerca del setpoint, usa parametros agresivos
    rollPID.SetTunings(RollaggKp, RollaggKi, RollaggKd);
  }
  rollPID.Compute();

  /**
  //Serial.println(pitchOutput);
  Serial.println("Pitch: ");
  Serial.println(mpuPitch);
  Serial.println("Roll: ");
  Serial.println(mpuRoll);
  //Serial.println(rollOutput);
  **/
            
  if(bUpdateFlags & THROTTLE_FLAG) {
    if(servoMotorTL.readMicroseconds() && servoMotorTR.readMicroseconds() 
        && servoMotorBL.readMicroseconds() && servoMotorBR.readMicroseconds()
        != unThrottleIn) {
          outputTR = unThrottleIn;
          outputTL = unThrottleIn;
          outputBL = unThrottleIn;
          outputBR = unThrottleIn;
          auxTR = unThrottleIn;
          auxTL = unThrottleIn;
          auxBL = unThrottleIn;
          auxBR = unThrottleIn;
          /**
           * ESTABILIZADOR
           * AUTOMATICO
           */
          // Girar Izquierda (Regresa Grados Positivos)
          if(mpuPitch > 0) {
            outputTL = unThrottleIn + (pitchOutput + motorGain); // Convertir grados a RPMs
            outputBL = unThrottleIn + (pitchOutput + motorGain);
          }
          // Girar Derecha (Regresa Grados Negastivos)
          if(mpuPitch < 0) {
            outputTR = unThrottleIn + (pitchOutput + motorGain); // Convertir grados a RPMs
            outputBR = unThrottleIn + (pitchOutput + motorGain);
          }
          // Inclinar Adelante (Grados negativos)
          if(mpuRoll < 0) {
            outputTL = unThrottleIn + (rollOutput + motorGain); // Convertir grados a RPMs
            outputTR = unThrottleIn + (rollOutput + motorGain);
          }
          // Inclinar Atras (Grados Positivos)
          if(mpuRoll > 0) {
            outputBL = unThrottleIn + (rollOutput + motorGain); // Convertir grados a RPMs
            outputBR = unThrottleIn + (rollOutput + motorGain);
          }
          /**
           * FIN
           * ESTABILIZADOR
           * AUTOMATICO
           */
    }
  }
  /**
  if(bUpdateFlags & PITCH_FLAG) {
    if(servoMotorTL.readMicroseconds() && servoMotorTR.readMicroseconds() 
        && servoMotorBL.readMicroseconds() && servoMotorBR.readMicroseconds()
        != unPitchIn) {
         //Serial.println(unPitchIn);
            pitchInput = map(unPitchIn, 900, 2000, -30, 30); // Valor deseado (necesita convertirse a grados)
            pitchSetpoint = mpuPitch; // Valor deseado en grados
            double gap = abs(pitchSetpoint-pitchInput); //distance away from setpoint
            if(gap<5)
            {  //we're close to setpoint, use conservative tuning parameters
              pitchPID.SetTunings(PitchconsKp, PitchconsKi, PitchconsKd);
            }
            else
            {
               //we're far from setpoint, use aggressive tuning parameters
               pitchPID.SetTunings(PitchaggKp, PitchaggKi, PitchaggKd);
            }
          pitchPID.Compute();
          // Ir A Derecha
          if(unPitchIn > 1550) {
            outputTL = auxTL + (pitchOutput + motorGain); // Convertir grados a RPMs
            outputBL = auxTL + (pitchOutput + motorGain);
          }
          // Ir A La Izquierda
          if(unPitchIn < 1450) {
            outputTR = auxTR + (pitchOutput + motorGain);
            outputBR = auxBR + (pitchOutput + motorGain);
          }
    }
  }
  
  if(bUpdateFlags & ROLL_FLAG) {
    if(servoMotorTL.readMicroseconds() && servoMotorTR.readMicroseconds() 
        && servoMotorBL.readMicroseconds() && servoMotorBR.readMicroseconds()
        != unRollIn) {
          //Serial.println(unRollIn);
            rollInput = map(unRollIn, 900, 2000, -30, 30); // Valor deseado (necesita convertirse a grados)
            rollSetpoint = mpuRoll; // Valor deseado en grados
            double gap = abs(rollSetpoint-rollInput); //distance away from setpoint
            if(gap<5)
            {  //we're close to setpoint, use conservative tuning parameters
              rollPID.SetTunings(RollconsKp, RollconsKi, RollconsKd);
            }
            else
            {
               //we're far from setpoint, use aggressive tuning parameters
               rollPID.SetTunings(RollaggKp, RollaggKi, RollaggKd);
            }
          pitchPID.Compute();
          if(unRollIn > 1550) {
            outputTL = auxTL + (rollOutput + motorGain);
            outputTR = auxTR + (rollOutput + motorGain);
          }
          if(unRollIn < 1450) {
            outputBL = auxBL + (rollOutput + motorGain);
            outputBR = auxBR + (rollOutput + motorGain);
          }
    }
  }
  **/
    
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

void calcRoll() {
  if(digitalRead(ROLL_IN_PIN) == HIGH) { 
    ulRollStart = micros();
  } else{
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
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
