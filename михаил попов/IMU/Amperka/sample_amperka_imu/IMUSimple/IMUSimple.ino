// Библиотека для работы с модулями IMU
#include <TroykaIMU.h>

// Создаём объект для работы с гироскопом
Gyroscope gyroscope;
// Создаём объект для работы с акселерометром
Accelerometer accelerometer;

void setup() {
    // Открываем последовательный порт
    Serial.begin(115200);
    // Выводим сообщение о начале инициализации
    Serial.println("IMU Begin");
    // Инициализируем гироскоп
    gyroscope.begin();
    // Инициализируем акселерометр
    accelerometer.begin();
    // Выводим сообщение об удачной инициализации
    Serial.println("Initialization completed");
    Serial.println("Gyroscope\t\t\tAccelerometer\t\t\t");
}

void loop() {
    // Выводим угловую скорость в градусах в секунду относительно оси X, Y и Z
    Serial.print(gyroscope.readRotationDegX());
    Serial.print("\t");
    Serial.print(gyroscope.readRotationDegY());
    Serial.print("\t");
    Serial.print(gyroscope.readRotationDegZ());
    Serial.print("\t\t");
    // Выводим направления и величины ускорения в м/с² относительно оси X, Y и Z
    Serial.print(accelerometer.readAccelerationAX());
    Serial.print("\t");
    Serial.print(accelerometer.readAccelerationAY());
    Serial.print("\t");
    Serial.print(accelerometer.readAccelerationAZ());
    Serial.println("\t\t");
    delay(100);
}
