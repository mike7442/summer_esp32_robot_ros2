#include <TroykaIMU.h>
Gyroscope gyroscope;
float axel_rotation[3] = {0,0,0};

void setup() {
    Serial.begin(115200);
    Serial.println("Gyroscope begin");
    gyroscope.begin();
    Serial.println("Initialization completed");
}
void axel_rotate(float* arr){
    arr[0] = gyroscope.readRotationDegX();
    arr[1] = gyroscope.readRotationDegY();
    arr[2] = gyroscope.readRotationDegZ();
}
void loop() {
    axel_rotate(axel_rotation);
    for(int i = 0;i<3;i++){
        Serial.print(axel_rotation[i]);
    }
    Serial.println();
    delay(100);
}
