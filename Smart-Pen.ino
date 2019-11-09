#include "Wire.h" // This library allows you to communicate with I2C devices.
const int ACCEL_CONFIG_REGISTER = 0x1C; // R/W
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelX, accelY, accelZ; // storage for acclerometer data;
int16_t gyroX, gyroY, gyroZ; // storage for gyroscope data;
float deltaTime, currentTime, previousTime = 0.0f;

char tmp_str[7]; // temporary variable used in convert function

float curr_x, curr_y, curr_z = 0; // variables for integration
float curr_vel_x, curr_vel_y, curr_vel_z = 0;
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
  CalculateIMUError();
  delay(20);
}



void loop() {
  UpdateTime();
  ReadAccelerationData();
  ReadGyroData();
  printRawSensorData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);



  if (isFirstLoop) {
    initial_z_accel = accelZ;
    isFirstLoop = false;
  }
  
/*
  if (abs(accelerometer_x - 0) < 500) {
    accelerometer_x = 0;
  }
  if (abs(accelerometer_y - 0) < 200) {
    accelerometer_y = 0;
  }
  if (abs(accelerometer_z - initial_z_accel) < 400) {
    accelerometer_z = initial_z_accel;
  }  */

  int16_t velocityX = deltaTime * accelX;
  int16_t velocityY = deltaTime * accelY;
  int16_t velocityZ = deltaTime * accelZ;

  curr_x += curr_vel_x*delta_time/1000 + 0.5*delta_time*delta_time*(accelerometer_x - 0)/1000000;
  curr_y += curr_vel_y*delta_time/1000 + 0.5*delta_time*delta_time*(accelerometer_y - 0)/1000000;
  curr_z += curr_vel_z*delta_time/1000 + 0.5*delta_time*delta_time*(accelerometer_z - initial_z_accel)/1000000;
  time = curr_time; */
}

int16_t accelErrorX, accelErrorY, accelErrorZ = 0;
float gyroErrorX, gyroErrorY, gyroErrorZ = 0;

int16_t sum = 0;
void CalculateIMUError() {
  int c = 0;
  while (c < 200) {
    sum += 1;
    ReadAccelerationData();
    accelErrorX += accelX;
    accelErrorY += accelY;
    Serial.println(int16ToString(accelErrorY + accelY));
    accelErrorZ += accelZ;

    ReadGyroData();
    gyroErrorX += (gyroX);
    gyroErrorY += (gyroY);
    gyroErrorZ += (gyroY);
    c++;
    printRawSensorData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  }
  
/*
  accelErrorX /= 200.0f;
  accelErrorY /= 200.0f;
  accelErrorZ /= 200.0f;
  gyroErrorX /= 200.0f;
  gyroErrorY /= 200.0f;
  gyroErrorZ /= 200.0f;
 */
  Serial.println("ERROR");
  printRawSensorData(accelErrorX, accelErrorY, accelErrorZ, gyroErrorX, gyroErrorY, gyroErrorZ);
}

const float ACCEL_DIVISOR = 16384.0f;
const float GYRO_DIVISOR = 131.0f;

void ReadAccelerationData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accelX = (Wire.read()<<8 | Wire.read()) / ACCEL_DIVISOR; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelY = (Wire.read()<<8 | Wire.read()) / ACCEL_DIVISOR; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelZ = (Wire.read()<<8 | Wire.read()) / ACCEL_DIVISOR; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
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


void printRawSensorData(int16_t accelerometer_x, int16_t accelerometer_y, int16_t accelerometer_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
  Serial.print("aX = "); Serial.print(int16ToString(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(int16ToString(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(int16ToString(accelerometer_z));
  
  Serial.print(" | gX = "); Serial.print(int16ToString(gyro_x));
  Serial.print(" | gY = "); Serial.print(int16ToString(gyro_y));
  Serial.print(" | gZ = "); Serial.print(int16ToString(gyro_z)); 
  Serial.println();
}

void UpdateTime() {
  previousTime = currentTime;
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000;
}

char* int16ToString(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
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
