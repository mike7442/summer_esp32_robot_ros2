#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Ваши WiFi настройки
const char* ssid = "delo";
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

    // Допустим, шлём 6 чисел, например, от 0 до 5
    String message;
    for (int i = 0; i < 6; i++) {
      message += String(i);
      if (i < 5) message += ",";
    }

    // Отправляем всем подключенным клиентам
    ws.textAll(message);
  }
}
