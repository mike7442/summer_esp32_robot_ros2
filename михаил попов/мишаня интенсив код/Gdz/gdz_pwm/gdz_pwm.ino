#include <Arduino.h>

// Пины для моторов
#define MOTOR1_A 32
#define MOTOR1_B 33
#define MOTOR2_A 26
#define MOTOR2_B 25

const int freq = 1000;     // Частота ШИМ, Гц
const int resolution = 8;  // 8 бит (0-255)
const int maxSpeed = 255;
const int minSpeed = 0;
const int step = 5;        // Шаг изменения скорости
const int delayMs = 100;    // Задержка между изменениями

void setup() {
  ledcAttach(MOTOR1_A, freq, resolution);
  ledcAttach(MOTOR1_B, freq, resolution);
  ledcAttach(MOTOR2_A, freq, resolution);
  ledcAttach(MOTOR2_B, freq, resolution);

  stopMotors();
}

void loop() {
  // Плавный разгон вперед
  for (int speed = minSpeed; speed <= maxSpeed; speed += step) {
    motor1Forward(speed);
    motor2Forward(speed);
    delay(delayMs);
  }
  delay(500);

  // Плавное торможение вперед
  for (int speed = maxSpeed; speed >= minSpeed; speed -= step) {
    motor1Forward(speed);
    motor2Forward(speed);
    delay(delayMs);
  }
  stopMotors();
  delay(1000);

  // Плавный разгон назад
  for (int speed = minSpeed; speed <= maxSpeed; speed += step) {
    motor1Backward(speed);
    motor2Backward(speed);
    delay(delayMs);
  }
  delay(500);

  // Плавное торможение назад
  for (int speed = maxSpeed; speed >= minSpeed; speed -= step) {
    motor1Backward(speed);
    motor2Backward(speed);
    delay(delayMs);
  }
  stopMotors();
  delay(1000);
}

void motor1Forward(int speed) {
  ledcWrite(MOTOR1_A, speed);
  ledcWrite(MOTOR1_B, 0);
}

void motor1Backward(int speed) {
  ledcWrite(MOTOR1_A, 0);
  ledcWrite(MOTOR1_B, speed);
}

void motor2Forward(int speed) {
  ledcWrite(MOTOR2_A, speed);
  ledcWrite(MOTOR2_B, 0);
}

void motor2Backward(int speed) {
  ledcWrite(MOTOR2_A, 0);
  ledcWrite(MOTOR2_B, speed);
}

void stopMotors() {
  ledcWrite(MOTOR1_A, 0);
  ledcWrite(MOTOR1_B, 0);
  ledcWrite(MOTOR2_A, 0);
  ledcWrite(MOTOR2_B, 0);
}