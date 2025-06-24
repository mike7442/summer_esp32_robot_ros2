#include <WiFi.h>

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
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
  }

  if (client && client.connected()) {
    static unsigned long lastSendTime = 0;
    unsigned long now = millis();

    if (now - lastSendTime > 10) {  // отправка каждые ~10мс
      lastSendTime = now;
      uint8_t buffer[24];
      pack6FloatsToBytes(1.23, -4.56, 7.89, 0.12, -3.45, 140.78, buffer);
      client.write(buffer, sizeof(buffer));
    }
  }
}
