#include <SoftwareSerial.h>
#include <Wire.h>

// ========================================================
// ARES-X6 Mars Rover (Arduino UNO + HC-05 + OV7670 FIFO)
// ========================================================
// NOTE:
// - Full camera capture on a bare OV7670 (no FIFO) is not practical on UNO.
// - This sketch supports OV7670 modules that include AL422 FIFO.
// - Rover features (drive/sensors/voice) remain integrated with camera commands.

// HC-05 Bluetooth
const uint8_t BT_RX = 10; // UNO receives on D10 (connect to HC-05 TX)
const uint8_t BT_TX = 11; // UNO transmits on D11 (connect to HC-05 RX via divider)
SoftwareSerial bt(BT_RX, BT_TX);

// Motor driver pins (L298N style)
const uint8_t ENA = 5;
const uint8_t IN1 = 2;
const uint8_t IN2 = 3;
const uint8_t ENB = 6;
const uint8_t IN3 = 4;
const uint8_t IN4 = 7;

// Ultrasonic sensor (HC-SR04)
const uint8_t TRIG_PIN = 12;
const uint8_t ECHO_PIN = 13;

// Soil moisture sensor
const uint8_t SOIL_PIN = A0;

// ---------------- CAMERA (OV7670 + AL422 FIFO) ----------------
// SCCB (OV7670 I2C-like control) uses A4/A5 on UNO via Wire.
const uint8_t OV7670_ADDR = 0x21;

// FIFO control pins (adjust to your module wiring)
const uint8_t CAM_WRST = A1;
const uint8_t CAM_RRST = A2;
const uint8_t CAM_WE = A3;
const uint8_t CAM_RCK = 8;
const uint8_t CAM_OE = 9;

// FIFO data bus from camera module to UNO (8-bit)
const uint8_t CAM_D0 = A4; // If this conflicts with Wire on your board/module,
const uint8_t CAM_D1 = A5; // rewire and change pins accordingly.
const uint8_t CAM_D2 = A6; // A6/A7 exist on Nano; not UNO DIP.
const uint8_t CAM_D3 = A7;
const uint8_t CAM_D4 = 14; // D14 == A0 on some cores; update if needed.
const uint8_t CAM_D5 = 15;
const uint8_t CAM_D6 = 16;
const uint8_t CAM_D7 = 17;

// Rover settings
int motorSpeed = 180;            // 0-255
long obstacleThresholdCm = 25;   // auto-stop distance
bool autoAvoidEnabled = true;
bool readingsEnabled = false;
unsigned long lastReadingsMs = 0;
const unsigned long readingsIntervalMs = 2000;

char commandBuffer[48];
uint8_t commandLen = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(CAM_WRST, OUTPUT);
  pinMode(CAM_RRST, OUTPUT);
  pinMode(CAM_WE, OUTPUT);
  pinMode(CAM_RCK, OUTPUT);
  pinMode(CAM_OE, OUTPUT);

  Serial.begin(115200);
  bt.begin(9600);
  Wire.begin();

  initCamera();
  stopRover();

  Serial.println(F("ARES-X6 ready."));
  bt.println(F("ARES-X6 ready."));
  bt.println(F("Drive: F,B,L,R,X,STOP,SPEED n"));
  bt.println(F("Sensors: START READINGS,STOP READINGS,AUTO ON,AUTO OFF,STATUS"));
  bt.println(F("Camera: CAM SNAP, CAM DUMP"));
}

void loop() {
  readBluetoothCommands();

  if (autoAvoidEnabled && isObstacleTooClose()) {
    stopRover();
    bt.println(F("Obstacle detected -> STOP"));
  }

  if (readingsEnabled && millis() - lastReadingsMs >= readingsIntervalMs) {
    publishSensorReadings();
    lastReadingsMs = millis();
  }
}

void readBluetoothCommands() {
  while (bt.available()) {
    char c = (char)bt.read();
    if (c == '\n' || c == '\r') {
      if (commandLen > 0) {
        commandBuffer[commandLen] = '\0';
        normalizeCommand(commandBuffer);
        handleCommand(commandBuffer);
        commandLen = 0;
      }
    } else if (commandLen < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLen++] = c;
    }
  }
}

void normalizeCommand(char* s) {
  // Trim leading/trailing spaces and uppercase in-place.
  uint8_t start = 0;
  while (s[start] == ' ') start++;

  uint8_t end = 0;
  while (s[end] != '\0') end++;
  while (end > start && s[end - 1] == ' ') end--;

  uint8_t j = 0;
  for (uint8_t i = start; i < end; i++) {
    char c = s[i];
    if (c >= 'a' && c <= 'z') c -= 32;
    s[j++] = c;
  }
  s[j] = '\0';
}

bool startsWith(const char* text, const char* prefix) {
  while (*prefix) {
    if (*text++ != *prefix++) return false;
  }
  return true;
}

void handleCommand(char* cmd) {
  if (strcmp(cmd, "F") == 0 || strcmp(cmd, "MOVE FORWARD") == 0) { moveForward(); return; }
  if (strcmp(cmd, "B") == 0 || strcmp(cmd, "MOVE BACKWARD") == 0) { moveBackward(); return; }
  if (strcmp(cmd, "L") == 0 || strcmp(cmd, "TURN LEFT") == 0) { turnLeft(); return; }
  if (strcmp(cmd, "R") == 0 || strcmp(cmd, "TURN RIGHT") == 0) { turnRight(); return; }
  if (strcmp(cmd, "X") == 0 || strcmp(cmd, "STOP") == 0) { stopRover(); return; }

  if (strcmp(cmd, "START READINGS") == 0) {
    readingsEnabled = true;
    bt.println(F("Sensor readings: ON"));
    return;
  }
  if (strcmp(cmd, "STOP READINGS") == 0) {
    readingsEnabled = false;
    bt.println(F("Sensor readings: OFF"));
    return;
  }
  if (strcmp(cmd, "AUTO ON") == 0) {
    autoAvoidEnabled = true;
    bt.println(F("Auto obstacle avoidance: ON"));
    return;
  }
  if (strcmp(cmd, "AUTO OFF") == 0) {
    autoAvoidEnabled = false;
    bt.println(F("Auto obstacle avoidance: OFF"));
    return;
  }
  if (startsWith(cmd, "SPEED ")) {
    int value = atoi(cmd + 6);
    motorSpeed = constrain(value, 0, 255);
    bt.print(F("Speed set to "));
    bt.println(motorSpeed);
    return;
  }
  if (strcmp(cmd, "STATUS") == 0) { publishStatus(); return; }

  if (strcmp(cmd, "CAM SNAP") == 0) {
    cameraSnap();
    bt.println(F("Camera snap done."));
    return;
  }
  if (strcmp(cmd, "CAM DUMP") == 0) {
    bt.println(F("CAM_BEGIN"));
    dumpFifoToSerial(512); // small sample to avoid BT flooding
    bt.println(F("CAM_END"));
    return;
  }

  bt.print(F("Unknown command: "));
  bt.println(cmd);
}

bool isObstacleTooClose() {
  long distance = readDistanceCm();
  if (distance <= 0) return false;
  return distance < obstacleThresholdCm;
}

long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1;
  return duration / 58;
}

int readSoilRaw() {
  return analogRead(SOIL_PIN);
}

void publishSensorReadings() {
  long distance = readDistanceCm();
  int soilRaw = readSoilRaw();

  bt.print(F("DIST_CM="));
  bt.print(distance);
  bt.print(F(", SOIL_RAW="));
  bt.println(soilRaw);

  Serial.print(F("DIST_CM="));
  Serial.print(distance);
  Serial.print(F(", SOIL_RAW="));
  Serial.println(soilRaw);
}

void publishStatus() {
  bt.print(F("speed="));
  bt.print(motorSpeed);
  bt.print(F(", auto="));
  bt.print(autoAvoidEnabled ? F("ON") : F("OFF"));
  bt.print(F(", readings="));
  bt.println(readingsEnabled ? F("ON") : F("OFF"));
}

void initCamera() {
  digitalWrite(CAM_OE, HIGH);   // disable FIFO output
  digitalWrite(CAM_WE, HIGH);   // stop writing
  digitalWrite(CAM_WRST, HIGH);
  digitalWrite(CAM_RRST, HIGH);
  digitalWrite(CAM_RCK, LOW);

  // Basic OV7670 register init for QVGA + RGB565 (common baseline)
  writeCamReg(0x12, 0x80); // reset
  delay(100);
  writeCamReg(0x12, 0x14); // QVGA, RGB
  writeCamReg(0x40, 0xD0); // RGB565
}

void writeCamReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(OV7670_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void cameraSnap() {
  // Reset FIFO write pointer
  digitalWrite(CAM_WRST, LOW);
  digitalWrite(CAM_WRST, HIGH);

  // Enable write to FIFO; camera pushes pixels into FIFO in hardware.
  digitalWrite(CAM_WE, LOW);
  delay(100); // capture window (tune based on frame rate)
  digitalWrite(CAM_WE, HIGH);

  // Reset read pointer
  digitalWrite(CAM_RRST, LOW);
  pulseRck();
  digitalWrite(CAM_RRST, HIGH);
}

void pulseRck() {
  digitalWrite(CAM_RCK, LOW);
  delayMicroseconds(1);
  digitalWrite(CAM_RCK, HIGH);
  delayMicroseconds(1);
}

uint8_t readFifoByte() {
  // Placeholder read bus: you must map actual 8 data lines to readable pins.
  // Many UNO boards do not expose A6/A7, so adjust wiring/hardware.
  uint8_t b = 0;
  b |= (digitalRead(CAM_D0) ? 1 : 0) << 0;
  b |= (digitalRead(CAM_D1) ? 1 : 0) << 1;
  b |= (digitalRead(CAM_D2) ? 1 : 0) << 2;
  b |= (digitalRead(CAM_D3) ? 1 : 0) << 3;
  b |= (digitalRead(CAM_D4) ? 1 : 0) << 4;
  b |= (digitalRead(CAM_D5) ? 1 : 0) << 5;
  b |= (digitalRead(CAM_D6) ? 1 : 0) << 6;
  b |= (digitalRead(CAM_D7) ? 1 : 0) << 7;
  pulseRck();
  return b;
}

void dumpFifoToSerial(uint16_t sampleBytes) {
  pinMode(CAM_D0, INPUT);
  pinMode(CAM_D1, INPUT);
  pinMode(CAM_D2, INPUT);
  pinMode(CAM_D3, INPUT);
  pinMode(CAM_D4, INPUT);
  pinMode(CAM_D5, INPUT);
  pinMode(CAM_D6, INPUT);
  pinMode(CAM_D7, INPUT);

  digitalWrite(CAM_OE, LOW); // enable FIFO output
  for (uint16_t i = 0; i < sampleBytes; i++) {
    uint8_t v = readFifoByte();
    if (v < 16) bt.print('0');
    bt.print(v, HEX);
    bt.print(' ');
  }
  bt.println();
  digitalWrite(CAM_OE, HIGH);
}

void moveForward() {
  if (autoAvoidEnabled && isObstacleTooClose()) {
    stopRover();
    bt.println(F("Blocked: obstacle ahead"));
    return;
  }
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopRover() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
