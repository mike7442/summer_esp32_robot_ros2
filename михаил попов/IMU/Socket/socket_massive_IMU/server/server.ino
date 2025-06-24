#include <WiFi.h>
#include <TroykaIMU.h>
Gyroscope gyroscope;
float axel_rotation[3] = {0,0,0};

const char* ssid = "robotx";
const char* password = "78914040";

WiFiServer server(12345);  // порт

void pack6FloatsToBytes(float f1, float f2, float f3, float f4, float f5, float f6, uint8_t *bytes) {
  float inputs[6] = { f1, f2, f3, f4, f5, f6 };

  for (int i = 0; i < 6; ++i) {
    union {
      float f;
      uint32_t i;
    } converter;

    converter.f = inputs[i];

    bytes[i * 4 + 0] = (converter.i >> 0) & 0xFF;
    bytes[i * 4 + 1] = (converter.i >> 8) & 0xFF;
    bytes[i * 4 + 2] = (converter.i >> 16) & 0xFF;
    bytes[i * 4 + 3] = (converter.i >> 24) & 0xFF;
  }
}

WiFiClient client;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("TCP сервер запущен");
  gyroscope.begin();
}
void axel_rotate(float* arr){
    arr[0] = gyroscope.readRotationDegX();
    arr[1] = gyroscope.readRotationDegY();
    arr[2] = gyroscope.readRotationDegZ();
}
void loop() {

  if (!client || !client.connected()) {
    client = server.available();
  }

  if (client && client.connected()) {
    static unsigned long lastSendTime = 0;
    unsigned long now = millis();

    if (now - lastSendTime > 10) { 
      axel_rotate(axel_rotation);
      lastSendTime = now;
      uint8_t buffer[24];
      pack6FloatsToBytes(0.00,0.00,0.00,axel_rotation[0],axel_rotation[1],axel_rotation[2], buffer);
      client.write(buffer, sizeof(buffer));
    }
  }
}
