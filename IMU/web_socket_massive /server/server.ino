#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <cstdint>

void pack6FloatsToBytes(float f1, float f2, float f3, float f4, float f5, float f6, uint8_t* bytes) {
    float inputs[6] = {f1, f2, f3, f4, f5, f6};

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

// Ваши WiFi настройки
const char* ssid = "robotx";
const char* password = "78914040";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

unsigned long lastSendTime = 0;
unsigned long now = 0;

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT){
    Serial.printf("Client connected: %u\n", client->id());
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("Client disconnected: %u\n", client->id());
  }
}

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
  Serial.println(WiFi.localIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.begin();
}

void loop() {
  ws.cleanupClients();

  now = millis();
  if (now - lastSendTime > 50) {
    lastSendTime = now;
    uint8_t storage[24];
    pack6FloatsToBytes(1.23, -4.56, 7.89, 0.12, -3.45, 140.78, storage);
    // Отправляем бинарные данные всем клиентам WebSocket
    ws.binaryAll(storage, sizeof(storage));
  }
}
