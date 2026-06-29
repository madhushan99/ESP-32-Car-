#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// WiFi credentials
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

// HiveMQ Cloud credentials
const char *mqtt_server = "b1af481d3e2a4ec4b90301e5f0d2ad8c.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char *mqtt_username = "YOUR_MQTT_Username";
const char *mqtt_password = "YOUR_MQTT_PASSWORD";

// MQTT topics
const char *control_topic = "car/control";
const char *feedback_topic = "car/feedback";

// Motor pins
const int MOTOR_LEFT_RIGHT_FORWARD = 23;
const int MOTOR_LEFT_RIGHT_BACKWARD = 22;
const int MOTOR_STEER_LEFT = 21;
const int MOTOR_STEER_RIGHT = 19;
const int motor_speed_pin = 18;

// Ultrasonic & Servo pins
const int servoPin = 27;
const int trigPin = 32;
const int echoPin = 34;

// PWM Settings
const int freq = 5000;
const int resolution = 10;

WiFiClientSecure espClient;
PubSubClient client(espClient);
Servo myServo;

bool readyForCommands = false;
int currentGear = 1;
bool isMoving = false;

// ── Obstacle Avoidance ──────────────────────────────────────────────────────
volatile bool obstacleAvoidanceMode = false;
volatile bool obstacleInFront = false;
volatile bool oaTaskReady = false;
const float SAFE_DISTANCE = 35.0;

// ── Heartbeat Watchdog ──────────────────────────────────────────────────────
// Web sends "HB" every 500 ms.
// If no HB arrives for HEARTBEAT_TIMEOUT ms, Core 0 triggers an emergency stop.
volatile unsigned long lastHeartbeatMs = 0;   // Updated in mqttCallback (Core 1)
volatile bool heartbeatArmed = false;         // True after first HB received
const unsigned long HEARTBEAT_TIMEOUT = 1500; // 1500 ms = 3 missed beats

// Task handle for Core 0
TaskHandle_t ObstacleTask;

// ==================== CORE 0: OBSTACLE DETECTION + HEARTBEAT WATCHDOG =======
void obstacleDetectionTask(void *pvParameters)
{
  Serial.println("=== Core 0: Obstacle + Watchdog Task Started ===");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  // Servo setup on Core 0 timers 2 & 3
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);

  if (!myServo.attach(servoPin, 500, 2400))
  {
    Serial.println("ERROR: Servo attach failed!");
    oaTaskReady = false;
  }
  else
  {
    Serial.println("✅ Servo OK");
    myServo.write(90);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    oaTaskReady = true;
  }

  // Optimised sweep: 5 positions for speed
  int angles[] = {90, 70, 50, 110, 130};
  int angleIndex = 0;
  int currentAngle = 90;

  obstacleInFront = false;

  for (;;)
  {

    // ── HEARTBEAT WATCHDOG CHECK ────────────────────────────────────────────
    // Run every loop iteration — costs virtually nothing.
    if (heartbeatArmed)
    {
      unsigned long now = millis();
      unsigned long elapsed = now - lastHeartbeatMs;

      if (elapsed > HEARTBEAT_TIMEOUT)
      {
        // Connection lost — hard stop all motors immediately
        digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, LOW);
        digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, LOW);
        digitalWrite(MOTOR_STEER_LEFT, LOW);
        digitalWrite(MOTOR_STEER_RIGHT, LOW);
        ledcWrite(motor_speed_pin, 0);
        isMoving = false;

        // Keep logging every second until HB returns
        static unsigned long lastWdLog = 0;
        if (now - lastWdLog > 1000)
        {
          Serial.printf("💀 WATCHDOG: No HB for %lums — Emergency Stop!\n", elapsed);
          lastWdLog = now;
        }
        // NOTE: We don't publish feedback here because MQTT is likely down.
        // Once the connection recovers, heartbeatArmed resets and we resume.
        // Reset armed flag so we don't keep spamming stop after reconnect
        // until a fresh HB arrives.
        heartbeatArmed = false;
      }
    }
    // ── END WATCHDOG CHECK ──────────────────────────────────────────────────

    // ── OBSTACLE AVOIDANCE ──────────────────────────────────────────────────
    if (obstacleAvoidanceMode && oaTaskReady)
    {

      currentAngle = angles[angleIndex];
      myServo.write(currentAngle);
      vTaskDelay(80 / portTICK_PERIOD_MS); // Fast servo movement

      // Quick single-shot distance reading
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      long duration = pulseIn(echoPin, HIGH, 25000); // Short timeout

      if (duration > 0)
      {
        float distance = duration * 0.0343 / 2.0;

        if (distance > 2 && distance < SAFE_DISTANCE)
        {
          obstacleInFront = true;
        }
        else if (distance >= SAFE_DISTANCE || distance > 400)
        {
          if (currentAngle >= 70 && currentAngle <= 110)
          {
            obstacleInFront = false;
          }
        }
      }
      else
      {
        // No echo = clear path
        if (currentAngle >= 70 && currentAngle <= 110)
        {
          obstacleInFront = false;
        }
      }

      angleIndex = (angleIndex + 1) % 5;
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    else
    {
      // OA mode OFF — centre servo and idle
      if (oaTaskReady && currentAngle != 90)
      {
        myServo.write(90);
        currentAngle = 90;
        angleIndex = 0;
      }
      obstacleInFront = false;
      vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    // ── END OBSTACLE AVOIDANCE ──────────────────────────────────────────────
  }
}

// ==================== CORE 1: CAR CONTROL & MQTT ============================
void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== ESP32 Fast Car System (with Heartbeat Watchdog) ===\n");

  // Motor pins
  pinMode(MOTOR_LEFT_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_STEER_LEFT, OUTPUT);
  pinMode(MOTOR_STEER_RIGHT, OUTPUT);

  ledcAttach(motor_speed_pin, freq, resolution);
  ledcWrite(motor_speed_pin, 0);

  digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, LOW);
  digitalWrite(MOTOR_STEER_LEFT, LOW);
  digitalWrite(MOTOR_STEER_RIGHT, LOW);

  Serial.println("✅ Motors OK");

  connectWiFi();

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // Launch obstacle + watchdog task on Core 0, priority 2
  xTaskCreatePinnedToCore(
      obstacleDetectionTask,
      "OA_Watchdog",
      8000,
      NULL,
      2,
      &ObstacleTask,
      0 // Core 0
  );

  delay(1500);
  reconnectMQTT();

  Serial.println("\n✅ SYSTEM READY\n");
  Serial.println("Heartbeat watchdog: armed after first HB received.");
  Serial.println("Timeout: 1500 ms (3 missed beats)\n");
}

void loop()
{
  if (!client.connected())
  {
    // MQTT dropped — reset heartbeat so watchdog re-arms on reconnect
    heartbeatArmed = false;
    reconnectMQTT();
  }
  client.loop();

  // Obstacle avoidance motor override (Core 1 fast path)
  if (obstacleAvoidanceMode && obstacleInFront && isMoving)
  {
    if (digitalRead(MOTOR_LEFT_RIGHT_FORWARD) == HIGH)
    {
      ledcWrite(motor_speed_pin, 1023);
      digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, LOW);
      digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, HIGH);
      delay(100);
      digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, LOW);
      ledcWrite(motor_speed_pin, 0);
      isMoving = false;
      Serial.println("🛑 STOP: Obstacle detected");
      publishFeedback("🛑 Obstacle");
    }
  }

  delay(5); // Minimal delay for faster response
}

// ============================================================================
void connectWiFi()
{
  Serial.print("WiFi: ");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(" ✅");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println(" Failed");
  }
}

void reconnectMQTT()
{
  while (!client.connected())
  {
    Serial.print("MQTT: ");
    String clientId = "ESP32Car_" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("✅");
      client.subscribe(control_topic);
      delay(500);
      readyForCommands = true;
      publishFeedback("Ready");
    }
    else
    {
      Serial.print("Failed RC=");
      Serial.println(client.state());
      delay(3000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  if (!readyForCommands)
    return;

  String msg = "";
  for (unsigned int i = 0; i < length; i++)
    msg += (char)payload[i];
  if (msg.length() == 0)
    return;

  // ── HEARTBEAT ─────────────────────────────────────────────────────────────
  // Handle first and silently; do not log to keep Serial clean.
  if (msg == "HB")
  {
    lastHeartbeatMs = millis(); // Timestamp updated here on Core 1
    heartbeatArmed = true;      // Arm watchdog on Core 0
    return;
  }
  // ── END HEARTBEAT ──────────────────────────────────────────────────────────

  // Obstacle Avoidance Toggle
  if (msg == "OA_ON" || msg == "oa_on")
  {
    obstacleAvoidanceMode = true;
    obstacleInFront = false;
    Serial.println("🛡️ OA: ON");
    publishFeedback("🛡️ OA ON");
    return;
  }
  else if (msg == "OA_OFF" || msg == "oa_off")
  {
    obstacleAvoidanceMode = false;
    obstacleInFront = false;
    Serial.println("OA: OFF");
    publishFeedback("OA OFF");
    return;
  }

  // Gear Selection
  if (msg == "1")
  {
    currentGear = 1;
    if (isMoving)
      ledcWrite(motor_speed_pin, 600);
    Serial.println("⚙️ Gear 1");
  }
  else if (msg == "2")
  {
    currentGear = 2;
    if (isMoving)
      ledcWrite(motor_speed_pin, 741);
    Serial.println("⚙️ Gear 2");
  }
  else if (msg == "3")
  {
    currentGear = 3;
    if (isMoving)
      ledcWrite(motor_speed_pin, 882);
    Serial.println("⚙️ Gear 3");
  }
  else if (msg == "4")
  {
    currentGear = 4;
    if (isMoving)
      ledcWrite(motor_speed_pin, 1023);
    Serial.println("⚙️ Gear 4");
  }

  // Movement Commands
  if (msg == "F" || msg == "f")
  {
    if (obstacleAvoidanceMode && oaTaskReady && obstacleInFront)
    {
      Serial.println("⚠️ Blocked by obstacle");
      publishFeedback("⚠️ Blocked");
      return;
    }
    moveForward();
  }
  else if (msg == "B" || msg == "b")
  {
    moveBackward();
  }
  else if (msg == "L" || msg == "l")
  {
    turnLeft();
  }
  else if (msg == "R" || msg == "r")
  {
    turnRight();
  }
  else if (msg == "SF" || msg == "sf")
  {
    stopFrontMotors();
  }
  else if (msg == "SB" || msg == "sb")
  {
    stopBackMotors();
  }
  else if (msg == "SL" || msg == "sl")
  {
    stopTurningLeft();
  }
  else if (msg == "SR" || msg == "sr")
  {
    stopTurningRight();
  }
  else if (msg == "S" || msg == "s")
  {
    stopFrontMotors();
    stopTurningLeft();
    stopTurningRight();
  }
}

void publishFeedback(String msg)
{
  if (client.connected())
  {
    client.publish(feedback_topic, msg.c_str());
  }
}

void applyGearSpeed()
{
  int speed = 0;
  switch (currentGear)
  {
  case 1:
    speed = 600;
    break;
  case 2:
    speed = 741;
    break;
  case 3:
    speed = 882;
    break;
  case 4:
    speed = 1023;
    break;
  }
  ledcWrite(motor_speed_pin, speed);
}

void moveForward()
{
  isMoving = true;
  applyGearSpeed();
  digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, LOW);
  Serial.println("➡️ FWD G" + String(currentGear));
}

void moveBackward()
{
  isMoving = true;
  applyGearSpeed();
  digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, HIGH);
  Serial.println("⬅️ BACK G" + String(currentGear));
}

void turnLeft()
{
  digitalWrite(MOTOR_STEER_LEFT, HIGH);
  digitalWrite(MOTOR_STEER_RIGHT, LOW);
  Serial.println("↰ LEFT");
}

void turnRight()
{
  digitalWrite(MOTOR_STEER_LEFT, LOW);
  digitalWrite(MOTOR_STEER_RIGHT, HIGH);
  Serial.println("↱ RIGHT");
}

void stopFrontMotors()
{
  isMoving = false;
  ledcWrite(motor_speed_pin, 1023);
  digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, HIGH);
  delay(100);
  digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, LOW);
  ledcWrite(motor_speed_pin, 0);
  Serial.println("⏸️ STOP");
}

void stopBackMotors()
{
  isMoving = false;
  ledcWrite(motor_speed_pin, 1023);
  digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_RIGHT_BACKWARD, LOW);
  delay(100);
  digitalWrite(MOTOR_LEFT_RIGHT_FORWARD, LOW);
  ledcWrite(motor_speed_pin, 0);
  Serial.println("⏸️ STOP");
}

void stopTurningRight()
{
  digitalWrite(MOTOR_STEER_LEFT, HIGH);
  digitalWrite(MOTOR_STEER_RIGHT, LOW);
  delay(30);
  digitalWrite(MOTOR_STEER_LEFT, LOW);
  Serial.println("⏸️ STOP TURN");
}

void stopTurningLeft()
{
  digitalWrite(MOTOR_STEER_RIGHT, HIGH);
  digitalWrite(MOTOR_STEER_LEFT, LOW);
  delay(30);
  digitalWrite(MOTOR_STEER_RIGHT, LOW);
  Serial.println("⏸️ STOP TURN");
}
