#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ==== НАСТРОЙКИ WiFi ====
const char* ssid = "robotx";
const char* password = "78914040";

// ==== Пины моторов ====
#define MOTOR1_A 32
#define MOTOR1_B 33
#define MOTOR2_A 26
#define MOTOR2_B 25

const int freq = 1000;
const int resolution = 8;

// ==== Переменные для хранения скоростей ====
int speed1 = 0;
int speed2 = 0;

bool motor1Enabled = 0;
bool motor2Enabled = 0;
// ==== Веб-сервер на 80 порту ====
WebServer server(80);

void setup() {
  Serial.begin(115200);
  // Настроить WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected! IP: ");
  Serial.println(WiFi.localIP());

  // PWM пины
  ledcAttach(MOTOR1_A, freq, resolution);
  ledcAttach(MOTOR1_B, freq, resolution);
  ledcAttach(MOTOR2_A, freq, resolution);
  ledcAttach(MOTOR2_B, freq, resolution);

  stopMotors();

  // Обработчик главной страницы
  server.on("/", HTTP_GET, handleRoot);
  // Обработчик применения скоростей (POST)
  server.on("/set", HTTP_POST, handleSet);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

// ==== HTML интерфейс ====
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<div>
  <span class="label">Левый мотор:</span><br>
  <input type="range" min="0" max="255" value="%SPEED1%" name="скорость 1" class="slider" id="slider1"
    oninput="document.getElementById('val1').innerText=this.value">
  <span id="val1">%SPEED1%</span>
  <br>
  <input type="checkbox" id="enable1" name="enable1" checked>
  <label for="enable1">Реверс левого мотора</label>
</div>

<!-- Отступ между элементами -->
<div style="height: 30px;"></div>

<div>
  <span class="label">Правый мотор:</span><br>
  <input type="range" min="0" max="255" value="%SPEED2%" name="скорость 2" class="slider" id="slider2"
    oninput="document.getElementById('val2').innerText=this.value">
  <span id="val2">%SPEED2%</span>
  <br>
  <input type="checkbox" id="enable2" name="enable2" checked>
  <label for="enable2">Реверс правого мотора</label>
</div>
</html>
)rawliteral";

// ==== Обработчики ====
void handleRoot() {
  String html = HTML;
  html.replace("%SPEED1%", String(speed1));
  html.replace("%SPEED2%", String(speed2));
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("speed1")) speed1 = server.arg("speed1").toInt();
  if (server.hasArg("speed2")) speed2 = server.arg("speed2").toInt();

  if (server.hasArg("enable1")) {
  motor1Enabled = true;  // Чекбокс отмечен
  motor1Backward(speed1); 

} else {
  motor1Enabled = false;
  motor1Forward(speed1); // Чекбокс снят
}
if (server.hasArg("enable2")) {
  motor2Enabled = true;
  motor2Backward(speed1);
} else {
  motor2Enabled = false;
  motor2Forward(speed1);
}

  // После применения возвращаемся на главную
  handleRoot();
}

// ==== Моторы ====
void motor1Forward(int speed) {
  ledcWrite(MOTOR1_A, speed);
  ledcWrite(MOTOR1_B, 0);
}
void motor1Backward(int speed) {
  ledcWrite(MOTOR1_B, speed);
  ledcWrite(MOTOR1_A, 0);
}

void motor2Forward(int speed) {
  ledcWrite(MOTOR2_A, speed);
  ledcWrite(MOTOR2_B, 0);
}
void motor2Backward(int speed) {
  ledcWrite(MOTOR2_B, speed);
  ledcWrite(MOTOR2_A, 0);
}
void stopMotors() {
  ledcWrite(MOTOR1_A, 0);
  ledcWrite(MOTOR1_B, 0);
  ledcWrite(MOTOR2_A, 0);
  ledcWrite(MOTOR2_B, 0);
}