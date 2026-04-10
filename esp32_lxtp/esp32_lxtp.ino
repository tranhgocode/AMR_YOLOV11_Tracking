#include "BluetoothSerial.h"

// ================= CHỌN COMM TYPE =================
#define USE_SERIAL 1    // 1 = Serial USB, 0 = Bluetooth
// ====================================================

#if USE_SERIAL
  // Dùng Serial USB (UART0 mặc định)
  #define COMM Serial
#else
  // Dùng Bluetooth
  BluetoothSerial SerialBT;
  #define COMM SerialBT
#endif

/* ================= L298N ================= */
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENA 32
#define ENB 33

#define SPEED_NORMAL 200
#define SPEED_SLOW   120

/* ===== THỜI GIAN QUAY ===== */
#define TURN_90_MS   450
#define TURN_360_MS  1800

/* ================= STATE ================= */
bool allowMove = false;
bool pedestrianStop = false;
unsigned long pedestrianTime = 0;
int currentSpeed = SPEED_NORMAL;

/* ================= MOTOR ================= */
void stopMotor() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void leftRaw() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void rightRaw() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void applySpeed() {
  ledcWrite(0, currentSpeed);
  ledcWrite(1, currentSpeed);
}

/* ================= GÓC ================= */
void turnLeft90() {
  leftRaw();
  applySpeed();
  delay(TURN_90_MS);
  stopMotor();
}

void turnRight90() {
  rightRaw();
  applySpeed();
  delay(TURN_90_MS);
  stopMotor();
}

void turnAround360() {
  leftRaw();
  applySpeed();
  delay(TURN_360_MS);
  stopMotor();
}

/* ================= COMMAND ================= */
void processCommand(String cmd) {
  Serial.println("CMD: " + cmd);

  if (cmd == "STOP") {
    allowMove = false;
    pedestrianStop = false;
    stopMotor();
  }
  else if (cmd == "FORWARD") {
    allowMove = true;
    pedestrianStop = false;
    forward();
    applySpeed();
  }
  else if (cmd == "PEDESTRIAN") {
    stopMotor();
    pedestrianStop = true;
    pedestrianTime = millis();
  }
  else if (cmd == "SLOW_DOWN") {
    currentSpeed = SPEED_SLOW;
    if (allowMove) applySpeed();
  }
  else if (cmd == "SPEED_RESUME") {
    currentSpeed = SPEED_NORMAL;
    if (allowMove) applySpeed();
  }
  else if (cmd == "LEFT") {
    turnLeft90();
    forward();
    applySpeed();
  }
  else if (cmd == "RIGHT") {
    turnRight90();
    forward();
    applySpeed();
  }
  else if (cmd == "TURN_AROUND") {
    turnAround360();
    forward();
    applySpeed();
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);

#if USE_SERIAL
  Serial.println("✅ Serial USB Ready");
#else
  SerialBT.begin("ESP32_CAR");
  Serial.println("✅ Bluetooth Ready: ESP32_CAR");
#endif
}

/* ================= LOOP ================= */
void loop() {
  // Read single character (không cần nhấn Enter)
  if (COMM.available()) {
    char ch = COMM.read();
    
    if (ch == '1') {
      Serial.println("Quay trái");
      turnLeft90();
      allowMove = false;
    }
    else if (ch == '2') {
      Serial.println("Quay phải");
      turnRight90();
      allowMove = false;
    }
    else if (ch == '3') {
      Serial.println("Tiến");
      allowMove = true;
      forward();
      applySpeed();
    }
    else if (ch == '4') {
      Serial.println("Dừng");
      allowMove = false;
      stopMotor();
    }
  }

  // Pedestrian stop handling
  if (pedestrianStop) {
    if (millis() - pedestrianTime >= 4000) {
      pedestrianStop = false;
      if (allowMove) {
        forward();
        applySpeed();
      } else {
        stopMotor();
      }
    }
    return;
  }

  // Normal operation
  if (allowMove) {
    forward();
    applySpeed();
  } else {
    stopMotor();
  }
}