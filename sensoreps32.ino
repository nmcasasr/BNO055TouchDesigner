#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MicroOscUdp.h>

// =========== CONFIG WIFI ============
const char* ssid     = "______";
const char* password = "_______";

// =========== CONFIG OSC ============
IPAddress destIp(192, 168, 0, 7); // Cambia a la IP de tu PC con TouchDesigner
const uint16_t destPort = 9000;     // Puerto OSC en TD
const uint16_t localPort = 8888;    // Puerto local ESP32

WiFiUDP myUdp;
MicroOscUdp<1024> osc(&myUdp, destIp, destPort);

// =========== SENSOR ============
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

void setup() {
  Serial.begin(115200);
  delay(10);

  // ---- WiFi ----
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.println(WiFi.localIP());

  myUdp.begin(localPort);

  // ---- BNO055 ----
  if (!bno.begin()) {
    Serial.println("Error: No se detecta BNO055 :(");
    while (1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  // ---- Euler (orientación en grados) ----
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  osc.sendMessage("/orientation", "fff",
                  event.orientation.x,
                  event.orientation.y,
                  event.orientation.z);

  // ---- Quaternion ----
  imu::Quaternion q = bno.getQuat();
  osc.sendMessage("/quat", "ffff", q.w(), q.x(), q.y(), q.z());

  // ---- Gyroscope (rad/s) ----
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  osc.sendMessage("/gyro", "fff", gyro.x(), gyro.y(), gyro.z());

  // ---- Accelerometer (m/s^2) ----
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  osc.sendMessage("/accel", "fff", accel.x(), accel.y(), accel.z());

  // ---- Linear Accel (sin gravedad) ----
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  osc.sendMessage("/linaccel", "fff", linAccel.x(), linAccel.y(), linAccel.z());

  // ---- Gravity ----
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  osc.sendMessage("/gravity", "fff", gravity.x(), gravity.y(), gravity.z());

  // ---- Calibración ----
  uint8_t sys, gyroCal, accelCal, magCal;
  bno.getCalibration(&sys, &gyroCal, &accelCal, &magCal);
  osc.sendMessage("/calib", "iiii",
                  (int32_t)sys,
                  (int32_t)gyroCal,
                  (int32_t)accelCal,
                  (int32_t)magCal);

  // Debug opcional
  Serial.printf("Euler: %.2f %.2f %.2f | Quat: %.2f %.2f %.2f %.2f | "
                "Gyro: %.2f %.2f %.2f | Accel: %.2f %.2f %.2f | Lin: %.2f %.2f %.2f | Grav: %.2f %.2f %.2f | Calib: %d %d %d %d\n",
                event.orientation.x, event.orientation.y, event.orientation.z,
                q.w(), q.x(), q.y(), q.z(),
                gyro.x(), gyro.y(), gyro.z(),
                accel.x(), accel.y(), accel.z(),
                linAccel.x(), linAccel.y(), linAccel.z(),
                gravity.x(), gravity.y(), gravity.z(),
                sys, gyroCal, accelCal, magCal);

  delay(50); // ~20 Hz
}
