#include "Wire.h" // This library allows you to communicate with I2C devices.
const int ACCEL_CONFIG_REGISTER = 0x1C; // R/W
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
const float ACCEL_DIVISOR = 16384.0f;
const float GYRO_DIVISOR = 1; //c 131.0f;
const float GRAVITY_ACCEL = 9.8f;

float accelX, accelY, accelZ; // storage for acclerometer data;
float gyroX, gyroY, gyroZ; // storage for gyroscope data;
float deltaTime, currentTime, previousTime = 0.0f;
float velocityX, velocityY, velocityZ = 0;

float curr_x, curr_y, curr_z = 0; // variables for integration
float initial_z_accel = 0;
bool isFirstLoop = true;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  currentTime = millis();
  //CalculateIMUError();
 // delay(20);
}



void loop() {
  UpdateTime();
  ReadAccelerationData();
  Serial.print("aX = "); Serial.print(String(accelX));
  Serial.print(" | aY = "); Serial.print(String(accelY));
  Serial.print(" | aZ = "); Serial.print(String(accelZ));

  if (isFirstLoop) {
    initial_z_accel = accelZ;
    isFirstLoop = false;
  }
  
  velocityX += deltaTime * accelX;
  velocityY += deltaTime * accelY;
  velocityZ += deltaTime * accelZ;

  curr_x += velocityX*deltaTime + 0.5*deltaTime*deltaTime*(accelX - 0);
  curr_y += velocityY*deltaTime + 0.5*deltaTime*deltaTime*(accelY - 0);
  curr_z += velocityZ*deltaTime + 0.5*deltaTime*deltaTime*(accelZ);

  Serial.print("pX = "); Serial.print(String(curr_x));
  Serial.print(" | pY = "); Serial.print(String(curr_y));
  Serial.print(" | pZ = "); Serial.print(String(curr_z));
  Serial.println(); 
}

float accelErrorX, accelErrorY, accelErrorZ = 0;
float gyroErrorX, gyroErrorY, gyroErrorZ = 0;

void ReadAccelerationData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t tempAX = (int16_t) Wire.read()<<8;
  accelX = (float) (tempAX |= Wire.read());
  accelX = accelX*GRAVITY_ACCEL/ACCEL_DIVISOR;
  
  int16_t tempAY = (int16_t) Wire.read()<<8;
  accelY = (float) (tempAY |= Wire.read());
  accelY = accelY*GRAVITY_ACCEL/ACCEL_DIVISOR;
  
  int16_t tempAZ = (int16_t) Wire.read()<<8;
  accelZ = (float) (tempAZ |= Wire.read());
  accelZ = accelZ*GRAVITY_ACCEL/ACCEL_DIVISOR;
  
  accelZ -= initial_z_accel;
}

void ReadGyroData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accelX = (Wire.read()<<8 | Wire.read()) / GYRO_DIVISOR; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelY = (Wire.read()<<8 | Wire.read()) / GYRO_DIVISOR; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelZ = (Wire.read()<<8 | Wire.read()) / GYRO_DIVISOR; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
}


void PrintData() {
  Serial.print("aX = "); Serial.print(String(accelX));
  Serial.print(" | aY = "); Serial.print(String(accelY));
  Serial.print(" | aZ = "); Serial.print(String(accelZ));
}


void CalculateIMUError() {
}


void UpdateTime() {
  previousTime = currentTime;
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000;
}

double ConvertDegreesToRadians(double degrees) {
  double converter = 180.00 / PI;
  return degrees * converter;
}





























/* UNUSED FUNCTIONS */
void AngleHolder() {
      double powX = pow(accelX, 2);
    double powY = pow(accelY, 2);
    double powZ = pow(accelZ, 2);

    double degX = sqrt(powX + powZ);
    double degY = sqrt(powY + powZ);

    double radX = ConvertDegreesToRadians(degX);
    double radY = ConvertDegreesToRadians(degY);

    double atanX = atan(accelY / radX);
    double atanY = atan((-1 * accelX) / radY);
}

void setRegisterData(int16_t registerHexCode, int16_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registerHexCode);
  Wire.write(data);
  Wire.endTransmission(true);
}

void SetConfigurations() {
  /*
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x19); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(ACCEL_CONFIG_REGISTER); // PWR_MGMT_1 register
  Wire.write(2); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); */
}
