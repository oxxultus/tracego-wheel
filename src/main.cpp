#include <AFMotor.h>
#include <Arduino.h>
#include <Config.h>
#include <SoftwareSerial.h>

AF_DCMotor motor_L(1);
AF_DCMotor motor_R(4);

float Kp = 30.0;
float Kd = 10.0;
int baseSpeed = 200; //150
int lastError = 0;
bool recovering = false;

// 센서 핀
#define SENSOR_LEFT_INNER  A0
#define SENSOR_LEFT_OUTER  A1
#define SENSOR_RIGHT_OUTER A4
#define SENSOR_RIGHT_INNER A5

SoftwareSerial comm(COMM_RX_PIN, COMM_TX_PIN);
bool isRunning = false;

void bot_begin();
void bot_update();
void bot_resume();
void bot_pause();
void waitAndHandle();
void modulsSetting();

void setup() {
  modulsSetting();
  Serial.println("[DELIVERY BOT - SUB] 보조 모듈 준비 완료");
}

void loop() {
  waitAndHandle();
  bot_update();
}

void modulsSetting() {
  Serial.begin(115200);
  comm.begin(BAUD_RATE);
  bot_begin();
  Serial.println("[INFO] 모든 로컬 모듈 초기화 완료");
}

void waitAndHandle() {
  if (comm.available()) {
    String command = comm.readStringUntil('\n');
    command.trim();

    Serial.print("[수신] 명령어: ");
    Serial.println(command);
    comm.println("ACK");

    if (command == "START" || command == "GO") {
      Serial.println("[ACTION] 로봇 재시작");
      bot_resume();
    } else if (command == "STOP") {
      Serial.println("[ACTION] 로봇 정지");
      bot_pause();
    } else if (command == "TEST") {
      Serial.println("[ACTION] 작동시작");
      bot_resume();
      return;
    } else {
      Serial.println("[WARNING] 알 수 없는 명령");
    }
  }
}

void bot_begin() {
  motor_L.setSpeed(baseSpeed);
  motor_R.setSpeed(baseSpeed);
  motor_L.run(RELEASE);
  motor_R.run(RELEASE);
  isRunning = false;
}

void bot_resume() {
  isRunning = true;
}

void bot_pause() {
  isRunning = false;
  motor_L.run(RELEASE);
  motor_R.run(RELEASE);
}

void bot_update() {
  if (!isRunning) return;

  int s0 = analogRead(SENSOR_LEFT_OUTER);
  int s1 = analogRead(SENSOR_LEFT_INNER);
  int s2 = analogRead(SENSOR_RIGHT_INNER);
  int s3 = analogRead(SENSOR_RIGHT_OUTER);
  int threshold = 500;

  // 탈선 감지
  bool leftOff = (s0 < threshold) && (s1 < threshold);
  bool rightOff = (s2 < threshold) && (s3 < threshold);

  if (!recovering && (leftOff || rightOff)) {
    recovering = true;
    Serial.println("==> 탈선 감지: 복귀 모드 진입");
  }

  if (recovering) {
    int error = 0;
    if (leftOff && !rightOff) error = -2;
    else if (!leftOff && rightOff) error = 2;
    else error = 0;

    int derivative = error - lastError;
    int correction = Kp * error + Kd * derivative;
    lastError = error;

    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motor_L.setSpeed(leftSpeed);
    motor_R.setSpeed(rightSpeed);
    motor_L.run(FORWARD);
    motor_R.run(FORWARD);

    // 복귀 완료
    if (s0 > threshold && s1 > threshold && s2 > threshold && s3 > threshold) {
      recovering = false;
      Serial.println("==> 복귀 완료: 일반 주행 모드 전환");
    }

    delay(10);
    return;
  }

  // === 일반 PID 주행 ===
  int weights[4] = {-3, -1, 1, 3};
  int sensors[4] = {s0, s1, s2, s3};

  long sum = 0;
  long weightedSum = 0;
  for (int i = 0; i < 4; i++) {
    int value = (sensors[i] < threshold) ? 1000 : 0; // 흑선에 있을 때만 감지
    sum += value;
    weightedSum += value * weights[i];
  }

  if (sum == 0) {
    motor_L.run(RELEASE);
    motor_R.run(RELEASE);
    Serial.println("==> 선을 놓침: 정지");
    return;
  }

  int error = weightedSum / sum;
  int derivative = error - lastError;
  int correction = Kp * error + Kd * derivative;
  correction = constrain(correction, -100, 100);  // 보정 한계
  lastError = error;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motor_L.setSpeed(leftSpeed);
  motor_R.setSpeed(rightSpeed);
  motor_L.run(FORWARD);
  motor_R.run(FORWARD);

  // 디버깅
  Serial.print("S0:"); Serial.print(s0);
  Serial.print(" S1:"); Serial.print(s1);
  Serial.print(" S2:"); Serial.print(s2);
  Serial.print(" S3:"); Serial.print(s3);
  Serial.print(" E:"); Serial.print(error);
  Serial.print(" C:"); Serial.println(correction);
}