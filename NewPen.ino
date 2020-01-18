#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include "Wire.h"

#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

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


volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Initializing I2C devices...");
  
  mpu.initialize();
  pinMode(2, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(2));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP Ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP INITIALIZATION FAILED (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(13, OUTPUT);
  
  
}


void loop() {
  if (!dmpReady) {
    return;
  }

  while (!mpuInterrupt && fifoCount < packetSize) {

    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

//  
  while (!Serial.available()) {}
//  
    
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
  }
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      while(fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }
      
      if (Serial.available()) {
        char input;
        input = Serial.read();
        if (input == '.') {
            digitalWrite(13, HIGH);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print(aa.x);
            Serial.print(", ");
            Serial.print(aa.y);
            Serial.print(", ");
            Serial.print(aa.z);
            Serial.print(", ");
            Serial.print(q.w, 2);
            Serial.print(", ");
            Serial.print(q.x, 2);
            Serial.print(", ");
            Serial.print(q.y, 2);
            Serial.print(", ");
            Serial.println(q.z, 2);
            digitalWrite(13, LOW);
        }
      }
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(13, blinkState);
  }
}
