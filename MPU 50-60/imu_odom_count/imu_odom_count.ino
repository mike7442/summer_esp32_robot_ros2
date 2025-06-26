#include <Wire.h>
#include "Kalman.h"

const uint8_t IMUAddress = 0x68;
const int i2cSDA = 21;
const int i2cSCL = 22;

// Калибровочные коэффициенты
const float accelScale = 16384.0; // ±2g
const float gyroScale = 131.0;    // ±250°/s

// Фильтры Калмана для ориентации
Kalman kalmanX, kalmanY, kalmanZ;

// Данные с датчика
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// Позиция и скорость
float velX = 0, velY = 0, velZ = 0;
float posX = 0, posY = 0, posZ = 0;

// Ориентация (в градусах)
float yaw = 0, pitch = 0, roll = 0;

uint32_t lastTime = 0;
float robotState[6]; // Массив для хранения состояния

void setup() {
  Serial.begin(115200);
  Wire.begin(i2cSDA, i2cSCL);
  
  // Инициализация MPU6050
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);
  
  // Настройки диапазонов
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C); Wire.write(0x00); // ±2g
  Wire.endTransmission(true);
  
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1B); Wire.write(0x00); // ±250°/s
  Wire.endTransmission(true);

  // Инициализация фильтров
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);
  kalmanZ.setAngle(0);

  delay(100);
}

void loop() {
  readIMU();
  float dt = (micros() - lastTime) / 1000000.0;
  lastTime = micros();

  calculateOrientation(dt);
  // calculatePosition(dt);
  
  // printRobotState();
  delay(50);
}

void readIMU() {
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 14, true);

  accX = (Wire.read() << 8 | Wire.read()) / accelScale;
  accY = (Wire.read() << 8 | Wire.read()) / accelScale;
  accZ = (Wire.read() << 8 | Wire.read()) / accelScale;

  Wire.read(); Wire.read(); // Пропуск температуры

  gyroX = (Wire.read() << 8 | Wire.read()) / gyroScale;
  gyroY = (Wire.read() << 8 | Wire.read()) / gyroScale;
  gyroZ = (Wire.read() << 8 | Wire.read()) / gyroScale;
}

void calculateOrientation(float dt) {
  // Расчет углов из акселерометра
  float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  float accRoll = atan2(accY, accZ) * RAD_TO_DEG;
  float accYaw = atan2(accX, accY) * RAD_TO_DEG;

  // Фильтрация Калмана
  pitch = kalmanX.getAngle(accPitch, gyroX, dt);
  roll = kalmanY.getAngle(accRoll, gyroY, dt);
  yaw = kalmanZ.getAngle(accYaw, gyroZ, dt);
}

void calculatePosition(float dt) {
  // Компенсация гравитации с учетом ориентации
  float gravityX = sin(roll * DEG_TO_RAD);
  float gravityY = -sin(pitch * DEG_TO_RAD);
  float gravityZ = -cos(pitch * DEG_TO_RAD) * cos(roll * DEG_TO_RAD);

  // "Чистое" ускорение (без гравитации)
  float linearAccX = accX - gravityX;
  float linearAccY = accY - gravityY;
  float linearAccZ = accZ - gravityZ;

  // Интегрирование ускорения
  velX += linearAccX * dt;
  velY += linearAccY * dt;
  velZ += linearAccZ * dt;

  posX += velX * dt;
  posY += velY * dt;
  posZ += velZ * dt;
}

void printRobotState() {
  Serial.print("Position: X=");
  Serial.print(posX, 2);
  Serial.print("m Y=");
  Serial.print(posY, 2);
  Serial.print("m Z=");
  Serial.print(posZ, 2);
  Serial.print("m | Orientation: Yaw=");
  Serial.print(yaw, 1);
  Serial.print("° Pitch=");
  Serial.print(pitch, 1);
  Serial.print("° Roll=");
  Serial.print(roll, 1);
  Serial.println("°");
}

void getRobotState(float* stateArray) {
  stateArray[0] = posX;    // X position
  stateArray[1] = posY;    // Y position
  stateArray[2] = posZ;    // Z position
  stateArray[3] = yaw;     // Yaw angle
  stateArray[4] = pitch;   // Pitch angle
  stateArray[5] = roll;    // Roll angle
}
