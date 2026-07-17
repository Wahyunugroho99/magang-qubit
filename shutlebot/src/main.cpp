/*
  ESP32 OmniBot Bluetooth + Motor Pelontar + Pengatur Sudut NEMA17 PG51 + TB6600 + 1 LIMIT SWITCH GPIO2 NC

  Hardware utama:
  - ESP32 DevKit / ESP32-WROOM-32 (Bluetooth Classic)
  - 4 x motor gearbox JGA25-370 untuk roda omni
  - 4 x driver BTS7960 / IBT-2 untuk roda
  - 1 x motor DC pelontar
  - 1 x driver BTS7960 / IBT-2 untuk motor pelontar
  - 1 x stepper NEMA17 17HS8401S-PG51 untuk pengatur sudut
  - 1 x driver TB6600 untuk motor sudut NEMA17

  ============================================================
  PERINTAH BLUETOOTH
  ============================================================

  Gerakan robot:
    F = maju
    B = mundur
    L = geser kiri
    R = geser kanan
    G = diagonal maju-kiri
    I = diagonal maju-kanan
    H = diagonal mundur-kiri
    J = diagonal mundur-kanan
    Q = putar kiri / CCW
    E = putar kanan / CW
    S = berhenti gerakan roda

  Kecepatan roda:
    0..9 = level kecepatan roda
    +    = tambah kecepatan roda
    -    = kurangi kecepatan roda

  Motor pelontar:
    K      = hidupkan pelontar
    M      = matikan pelontar langsung
    P0..P9 = level kecepatan pelontar
    ]      = tambah kecepatan pelontar
    [      = kurangi kecepatan pelontar

  Pengatur sudut pelontar:
    Z       = homing ke limit switch sudut minimum
    A<nilai> = menuju sudut absolut, contoh A30 atau A43
               Akhiri dengan Enter, #, atau tunggu sekitar 150 ms.
    U       = naikkan sudut 1 derajat
    D       = turunkan sudut 1 derajat
    C       = hentikan gerakan sudut langsung
    V       = tampilkan status sudut

  Keselamatan:
    X = emergency stop roda, motor pelontar, dan motor sudut

  ============================================================
  PIN PENGATUR SUDUT
  ============================================================

  Driver TB6600:
    PUL+ / STEP+ = GPIO13
    PUL- / STEP- = GND ESP32
    DIR+         = GPIO15
    DIR-         = GND ESP32
    ENA+ dan ENA- tidak perlu disambungkan.
    Jika ENA tidak disambungkan, TB6600 biasanya aktif terus
    sehingga motor tetap menahan sudut.

  Limit switch yang dipakai hanya 1:
    Sudut minimum / posisi bawah / posisi 0 derajat = GPIO2

  Mode switch yang disarankan: NC (Normally Closed).
  Wiring:
    GPIO2 -> COM switch
    GND   -> NC switch

  Tidak perlu resistor pull-up eksternal karena program memakai
  INPUT_PULLUP internal ESP32.
  Dengan NC:
    - Belum tersentuh / normal = GPIO2 LOW
    - Switch tersentuh / kabel putus = GPIO2 HIGH
    - Limit dianggap aktif saat HIGH

  Catatan:
  GPIO2 adalah pin strapping/boot pada ESP32. Bisa dipakai, tetapi
  jika ESP32 susah upload/boot, pindahkan ke GPIO lain yang aman.

  Tidak ada limit switch maksimum.
  Batas atas hanya memakai batas software ANGLE_MAX_DEG.

  ============================================================
  CATATAN
  ============================================================

  - Instal library "AccelStepper" melalui Arduino Library Manager.
  - Program default untuk TB6600 microstep 1/8, motor 1.8 derajat,
    dan gearbox 51:1.
  - Sesuaikan MICROSTEPS dan GEAR_RATIO dengan setelan driver serta
    gearbox yang sebenarnya.
  - Saat ESP32 menyala, motor sudut otomatis melakukan homing ke limit switch bawah.
  - Semua GND ESP32, BTS7960, driver stepper, buck converter, dan
    sumber daya harus disatukan.
*/

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include "BluetoothSerial.h"
#include <AccelStepper.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth tidak aktif pada board/core yang dipilih.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Serial SPP tidak tersedia. Gunakan ESP32 klasik/WROOM-32.
#endif

BluetoothSerial SerialBT;

// ============================================================
// KOMUNIKASI UART KE ESPServo
// ============================================================
// Wiring silang:
//   ShuttleBot TX GPIO0  -> ESPServo RX2 GPIO16
//   ShuttleBot RX GPIO34 -> ESPServo TX2 GPIO17
//   GND ShuttleBot      -> GND ESPServo
// Catatan: GPIO0 harus tetap HIGH saat boot/upload ESP32.
constexpr uint8_t SERVO_UART_RX_PIN = 3;
constexpr uint8_t SERVO_UART_TX_PIN = 1;
constexpr uint32_t SERVO_UART_BAUD = 115200;

// ============================================================
// KONFIGURASI BTS7960
// ============================================================
struct MotorPins {
  uint8_t rpwm;
  uint8_t lpwm;
  uint8_t en;
  uint8_t rChannel;
  uint8_t lChannel;
  bool inverted;
};

// Motor roda dilihat dari atas:
//      FL -------- FR
//       |          |
//      RL -------- RR
MotorPins motorFL = {25, 26, 27, 0, 1, false};
MotorPins motorFR = {33, 32, 14, 2, 3, true};
MotorPins motorRL = {18, 19, 5, 4, 5, false};
MotorPins motorRR = {16, 17, 4, 6, 7, true};

// Driver BTS7960 untuk motor DC pelontar.
MotorPins motorLauncher = {21, 22, 23, 8, 9, false};

constexpr uint32_t PWM_FREQ = 10000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr int PWM_MAX = 255;

// Buzzer aktif: HIGH = bunyi, LOW = mati.
constexpr uint8_t BUZZER_PIN = 12;
constexpr uint32_t BUZZER_BOOT_MS = 300;
constexpr uint32_t BUZZER_CONNECT_MS = 500;
constexpr uint32_t BUZZER_WAIT_MS = 120;
constexpr uint32_t BUZZER_WAIT_INTERVAL_MS = 5000;
bool buzzerOn = false;
uint32_t buzzerOffAt = 0;
uint32_t lastBluetoothWaitBeepAt = 0;
bool bluetoothWasConnected = false;

// Kecepatan roda.
int driveSpeed = 180;
constexpr int DRIVE_SPEED_MIN = 70;
constexpr int DRIVE_SPEED_STEP = 15;

// Kecepatan motor DC pelontar.
int launcherSpeed = 170;
constexpr int LAUNCHER_SPEED_MIN = 80;
constexpr int LAUNCHER_SPEED_STEP = 15;
bool launcherRunning = false;

char lastMotionCommand = 'S';
bool waitingLauncherLevel = false;

// ============================================================
// KONFIGURASI MOTOR SUDUT NEMA17 + DRIVER STEP/DIR
// ============================================================
// Pin sinyal TB6600.
// Wiring yang dipakai:
//   GPIO13 -> PUL+ / STEP+
//   GPIO15 -> DIR+
//   GND    -> PUL- dan DIR-
// ENA+ dan ENA- boleh dikosongkan.
constexpr uint8_t ANGLE_STEP_PIN = 13;
constexpr uint8_t ANGLE_DIR_PIN = 15;

constexpr uint8_t ANGLE_MIN_LIMIT_PIN = 2;
constexpr uint8_t LIMIT_ACTIVE_STATE = HIGH;  // NC + INPUT_PULLUP: aktif saat switch terbuka

// Ubah menjadi true jika arah motor terbalik.
constexpr bool ANGLE_DIRECTION_INVERTED = false;

// Sesuaikan dengan motor dan DIP switch microstep driver.
constexpr float MOTOR_FULL_STEPS_PER_REV = 200.0f;  // motor 1.8 derajat
constexpr float MICROSTEPS = 8.0f;                  // TB6600 disetel 1/8 microstep
constexpr float GEAR_RATIO = 51.0f;                 // gearbox PG51

constexpr float STEPS_PER_OUTPUT_REV =
    MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;

constexpr float STEPS_PER_DEGREE =
    STEPS_PER_OUTPUT_REV / 360.0f;

// Batas mekanis pelontar. Ubah sesuai mekanisme robot.
constexpr float ANGLE_MIN_DEG = 0.0f;
constexpr float ANGLE_MAX_DEG = 43.0f;
constexpr float ANGLE_JOG_DEG = 1.0f;

// Kecepatan dinyatakan dalam pulsa STEP per detik.
constexpr float ANGLE_MAX_SPEED = 4000.0f;
constexpr float ANGLE_ACCELERATION = 7000.0f;
constexpr float ANGLE_HOME_SPEED = 1800.0f;
constexpr float ANGLE_HOME_ACCELERATION = 3000.0f;

constexpr float ANGLE_HOME_TRAVEL_DEG = 90.0f;
constexpr uint32_t ANGLE_HOME_TIMEOUT_MS = 20000;

// TB6600/DM542 menggunakan antarmuka STEP dan DIR.
AccelStepper angleStepper(
    AccelStepper::DRIVER,
    ANGLE_STEP_PIN,
    ANGLE_DIR_PIN
);

bool angleHomed = false;
bool angleHoming = false;
bool angleMovePending = false;
float requestedAngleDeg = 0.0f;
uint32_t angleHomeStartTime = 0;

// Parser nilai sudut A0 sampai A43.
bool waitingAngleValue = false;
char angleValueBuffer[16] = {};
uint8_t angleValueLength = 0;
uint32_t angleLastInputTime = 0;
constexpr uint32_t ANGLE_INPUT_TIMEOUT_MS = 150;

// Perintah yang diteruskan ke ESPServo.
// Y = conveyor ON, W = ambil 1x, T/O/N = servo manual.
const String validServoCommands = "YWTON?";

// Timeout gerakan roda opsional.
constexpr bool ENABLE_COMMAND_TIMEOUT = false;
constexpr uint32_t COMMAND_TIMEOUT_MS = 1200;
uint32_t lastCommandTime = 0;

// ============================================================
// FUNGSI BANTU
// ============================================================
long degreeToSteps(float degree) {
  return lroundf(degree * STEPS_PER_DEGREE);
}

float stepsToDegree(long steps) {
  return static_cast<float>(steps) / STEPS_PER_DEGREE;
}

bool minLimitActive() {
  return digitalRead(ANGLE_MIN_LIMIT_PIN) == LIMIT_ACTIVE_STATE;
}

void sendBoth(const String &message) {
  Serial.println(message);
  SerialBT.println(message);
}

void sendServoCommand(char command) {
  Serial2.write(command);
  Serial2.write('\n');
  sendBoth("ESPServo <= " + String(command));
}

void serviceServoUart() {
  while (Serial2.available()) {
    char input = static_cast<char>(Serial2.read());
    Serial.write(input);

    if (SerialBT.hasClient()) {
      SerialBT.write(input);
    }
  }
}

void startBuzzer(uint32_t durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOn = true;
  buzzerOffAt = millis() + durationMs;
}

void serviceBuzzer() {
  if (buzzerOn && static_cast<int32_t>(millis() - buzzerOffAt) >= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
  }
}

void serviceBluetoothBuzzer() {
  bool bluetoothConnected = SerialBT.hasClient();
  uint32_t now = millis();

  if (bluetoothConnected && !bluetoothWasConnected) {
    startBuzzer(BUZZER_CONNECT_MS);
  }

  if (!bluetoothConnected &&
      now - lastBluetoothWaitBeepAt >= BUZZER_WAIT_INTERVAL_MS) {
    startBuzzer(BUZZER_WAIT_MS);
    lastBluetoothWaitBeepAt = now;
  }

  bluetoothWasConnected = bluetoothConnected;
}

// ============================================================
// KONTROL BTS7960
// ============================================================
void setMotor(const MotorPins &motor, int power) {
  power = constrain(power, -PWM_MAX, PWM_MAX);

  // Saat power 0: tidak ada PWM dan driver dinonaktifkan.
  if (power == 0) {
    ledcWrite(motor.rpwm, 0);
    ledcWrite(motor.lpwm, 0);
    digitalWrite(motor.en, LOW);
    return;
  }

  if (motor.inverted) {
    power = -power;
  }

  digitalWrite(motor.en, HIGH);

  if (power > 0) {
    ledcWrite(motor.rpwm, power);
    ledcWrite(motor.lpwm, 0);
  } else {
    ledcWrite(motor.rpwm, 0);
    ledcWrite(motor.lpwm, -power);
  }
}

// ============================================================
// KONTROL RODA ROBOT
// ============================================================
void driveStop() {
  setMotor(motorFL, 0);
  setMotor(motorFR, 0);
  setMotor(motorRL, 0);
  setMotor(motorRR, 0);
  lastMotionCommand = 'S';
}

// x  : -1 kiri, +1 kanan
// y  : -1 mundur, +1 maju
// rot: -1 CCW, +1 CW
void driveCartesian(float x, float y, float rot) {
  float fl = y + x + rot;
  float fr = y - x - rot;
  float rl = y - x + rot;
  float rr = y + x - rot;

  float largest = max(max(fabs(fl), fabs(fr)), max(fabs(rl), fabs(rr)));
  if (largest < 1.0f) {
    largest = 1.0f;
  }

  fl /= largest;
  fr /= largest;
  rl /= largest;
  rr /= largest;

  setMotor(motorFL, lroundf(fl * driveSpeed));
  setMotor(motorFR, lroundf(fr * driveSpeed));
  setMotor(motorRL, lroundf(rl * driveSpeed));
  setMotor(motorRR, lroundf(rr * driveSpeed));
}

void executeMotion(char command) {
  switch (command) {
    case 'F': driveCartesian( 0.0f,  1.0f,  0.0f); break;
    case 'B': driveCartesian( 0.0f, -1.0f,  0.0f); break;
    case 'L': driveCartesian(-1.0f,  0.0f,  0.0f); break;
    case 'R': driveCartesian( 1.0f,  0.0f,  0.0f); break;

    case 'G': driveCartesian(-1.0f,  1.0f,  0.0f); break;
    case 'I': driveCartesian( 1.0f,  1.0f,  0.0f); break;
    case 'H': driveCartesian(-1.0f, -1.0f,  0.0f); break;
    case 'J': driveCartesian( 1.0f, -1.0f,  0.0f); break;

    case 'Q': driveCartesian( 0.0f,  0.0f, -1.0f); break;
    case 'E': driveCartesian( 0.0f,  0.0f,  1.0f); break;

    case 'S':
    default: driveStop(); break;
  }
}

// ============================================================
// KONTROL MOTOR DC PELONTAR
// ============================================================
void applyLauncherSpeed() {
  if (launcherRunning && launcherSpeed > 0) {
    setMotor(motorLauncher, launcherSpeed);
  } else {
    setMotor(motorLauncher, 0);
  }
}

void launcherStart() {
  if (launcherSpeed < LAUNCHER_SPEED_MIN) {
    launcherSpeed = LAUNCHER_SPEED_MIN;
  }

  launcherRunning = true;
  applyLauncherSpeed();
}

void launcherStop() {
  launcherRunning = false;
  launcherSpeed = 0;
  setMotor(motorLauncher, 0);
}

void setLauncherLevel(int level) {
  level = constrain(level, 0, 9);

  if (level == 0) {
    launcherStop();
  } else {
    launcherSpeed = map(level, 1, 9, LAUNCHER_SPEED_MIN, PWM_MAX);
    launcherRunning = true;
    applyLauncherSpeed();
  }

  String text = "Launcher PWM=" + String(launcherSpeed) +
                ", status=" + String(launcherRunning ? "ON" : "OFF");
  sendBoth(text);
}

// ============================================================
// KONTROL SUDUT PELONTAR
// ============================================================
void restoreNormalAngleMotionSettings() {
  angleStepper.setMaxSpeed(ANGLE_MAX_SPEED);
  angleStepper.setAcceleration(ANGLE_ACCELERATION);
}

void angleStopImmediate() {
  long current = angleStepper.currentPosition();

  // setCurrentPosition juga membuat kecepatan internal menjadi nol.
  angleStepper.setCurrentPosition(current);
  angleStepper.moveTo(current);

  angleMovePending = false;
  angleHoming = false;
}

void reportAngleStatus() {
  String status =
      "Sudut=" + String(stepsToDegree(angleStepper.currentPosition()), 2) +
      " deg, target=" + String(stepsToDegree(angleStepper.targetPosition()), 2) +
      " deg, homed=" + String(angleHomed ? "YA" : "TIDAK") +
      ", minLimit=" + String(minLimitActive() ? "AKTIF" : "OFF") +
      ", maxLimit=TIDAK ADA";

  sendBoth(status);
}

void finishHomingSuccess() {
  angleStepper.setCurrentPosition(degreeToSteps(ANGLE_MIN_DEG));
  angleStepper.moveTo(degreeToSteps(ANGLE_MIN_DEG));

  angleHoming = false;
  angleHomed = true;
  angleMovePending = false;

  restoreNormalAngleMotionSettings();
  sendBoth("Homing selesai. Sudut sekarang 0 derajat.");
}

void startAngleHoming() {
  angleStopImmediate();
  angleHomed = false;
  angleHoming = true;
  angleHomeStartTime = millis();

  angleStepper.setMaxSpeed(ANGLE_HOME_SPEED);
  angleStepper.setAcceleration(ANGLE_HOME_ACCELERATION);

  if (minLimitActive()) {
    finishHomingSuccess();
    return;
  }

  long travelSteps = degreeToSteps(ANGLE_HOME_TRAVEL_DEG);
  angleStepper.moveTo(angleStepper.currentPosition() - travelSteps);

  sendBoth("Homing sudut dimulai menuju limit switch bawah / 0 derajat.");
}

void setLauncherAngle(float targetDegree) {
  if (!angleHomed) {
    sendBoth("Sudut belum di-homing. Tekan/kirim Z terlebih dahulu.");
    return;
  }

  if (!isfinite(targetDegree)) {
    sendBoth("Nilai sudut tidak valid.");
    return;
  }

  targetDegree = constrain(targetDegree, ANGLE_MIN_DEG, ANGLE_MAX_DEG);

  restoreNormalAngleMotionSettings();

  requestedAngleDeg = targetDegree;
  angleStepper.moveTo(degreeToSteps(targetDegree));
  angleMovePending = true;

  sendBoth("Target sudut=" + String(targetDegree, 2) + " derajat.");
}

void jogLauncherAngle(float deltaDegree) {
  if (!angleHomed) {
    sendBoth("Sudut belum di-homing. Tekan/kirim Z terlebih dahulu.");
    return;
  }

  float baseDegree = stepsToDegree(angleStepper.targetPosition());
  setLauncherAngle(baseDegree + deltaDegree);
}

void serviceAngleMotor() {
  // Satu limit switch hanya untuk posisi bawah / 0 derajat.
  // Jika limit aktif saat motor bergerak turun, langsung berhenti
  // dan posisi disetel menjadi 0 derajat.
  if (minLimitActive()) {
    if (angleHoming) {
      finishHomingSuccess();
    } else if (angleStepper.distanceToGo() < 0 || angleStepper.speed() < 0.0f) {
      angleStepper.setCurrentPosition(degreeToSteps(ANGLE_MIN_DEG));
      angleStepper.moveTo(degreeToSteps(ANGLE_MIN_DEG));
      angleMovePending = false;
      angleHomed = true;
      restoreNormalAngleMotionSettings();
      sendBoth("Limit bawah aktif. Motor sudut berhenti dan posisi = 0 derajat.");
    }
  }

  if (angleHoming &&
      millis() - angleHomeStartTime > ANGLE_HOME_TIMEOUT_MS) {
    angleStopImmediate();
    angleHomed = false;
    restoreNormalAngleMotionSettings();
    sendBoth("Homing gagal: limit switch bawah tidak terdeteksi.");
    return;
  }

  angleStepper.run();

  if (angleMovePending && angleStepper.distanceToGo() == 0) {
    angleMovePending = false;
    sendBoth("Sudut tercapai=" +
             String(stepsToDegree(angleStepper.currentPosition()), 2) +
             " derajat.");
  }
}

// ============================================================
// PARSER PERINTAH SUDUT A<nilai>
// ============================================================
void resetAngleInput() {
  waitingAngleValue = false;
  angleValueLength = 0;
  angleValueBuffer[0] = '\0';
}

void finishAngleInput() {
  if (!waitingAngleValue) {
    return;
  }

  angleValueBuffer[angleValueLength] = '\0';

  if (angleValueLength == 0) {
    sendBoth("Format sudut salah. Contoh: A30");
    resetAngleInput();
    return;
  }

  char *endPointer = nullptr;
  float value = strtof(angleValueBuffer, &endPointer);

  if (endPointer == angleValueBuffer || *endPointer != '\0') {
    sendBoth("Nilai sudut tidak valid. Contoh: A45.5");
    resetAngleInput();
    return;
  }

  resetAngleInput();
  setLauncherAngle(value);
}

// ============================================================
// EMERGENCY STOP
// ============================================================
void emergencyStop() {
  driveStop();
  launcherStop();
  angleStopImmediate();
  sendServoCommand('N');

  sendBoth("EMERGENCY STOP: roda, pelontar, dan pengatur sudut berhenti.");
}

// ============================================================
// PEMROSESAN PERINTAH SATU KARAKTER
// ============================================================
void processCommand(char command) {
  if (command == '\r' || command == '\n' ||
      command == ' ' || command == '\t') {
    return;
  }

  command = toupper(static_cast<unsigned char>(command));
  lastCommandTime = millis();

  // P diikuti angka 0..9 untuk kecepatan motor DC pelontar.
  if (waitingLauncherLevel) {
    waitingLauncherLevel = false;

    if (command >= '0' && command <= '9') {
      setLauncherLevel(command - '0');
      return;
    }

    sendBoth("Format pelontar salah. Gunakan P0 sampai P9.");
  }

  if (command == 'P') {
    waitingLauncherLevel = true;
    return;
  }

  if (validServoCommands.indexOf(command) >= 0) {
    sendServoCommand(command);
    return;
  }

  // Angka tanpa P mengatur kecepatan roda.
  if (command >= '0' && command <= '9') {
    int level = command - '0';

    if (level == 0) {
      driveSpeed = 0;
      driveStop();
    } else {
      driveSpeed = map(level, 1, 9, DRIVE_SPEED_MIN, PWM_MAX);

      if (lastMotionCommand != 'S') {
        executeMotion(lastMotionCommand);
      }
    }

    sendBoth("Drive PWM=" + String(driveSpeed));
    return;
  }

  if (command == '+') {
    driveSpeed = constrain(
        driveSpeed + DRIVE_SPEED_STEP,
        DRIVE_SPEED_MIN,
        PWM_MAX
    );

    if (lastMotionCommand != 'S') {
      executeMotion(lastMotionCommand);
    }

    sendBoth("Drive PWM=" + String(driveSpeed));
    return;
  }

  if (command == '-') {
    driveSpeed = constrain(
        driveSpeed - DRIVE_SPEED_STEP,
        DRIVE_SPEED_MIN,
        PWM_MAX
    );

    if (lastMotionCommand != 'S') {
      executeMotion(lastMotionCommand);
    }

    sendBoth("Drive PWM=" + String(driveSpeed));
    return;
  }

  // Motor DC pelontar.
  if (command == 'K') {
    launcherStart();
    sendBoth("Launcher ON, PWM=" + String(launcherSpeed));
    return;
  }

  if (command == 'M') {
    launcherStop();
    sendBoth("Launcher OFF");
    return;
  }

  if (command == ']') {
    if (launcherSpeed < LAUNCHER_SPEED_MIN) {
      launcherSpeed = LAUNCHER_SPEED_MIN;
    } else {
      launcherSpeed = constrain(
          launcherSpeed + LAUNCHER_SPEED_STEP,
          LAUNCHER_SPEED_MIN,
          PWM_MAX
      );
    }

    if (launcherRunning) {
      applyLauncherSpeed();
    }

    sendBoth("Launcher PWM=" + String(launcherSpeed));
    return;
  }

  if (command == '[') {
    if (launcherSpeed <= LAUNCHER_SPEED_MIN) {
      launcherStop();
    } else {
      launcherSpeed -= LAUNCHER_SPEED_STEP;

      if (launcherSpeed < LAUNCHER_SPEED_MIN) {
        launcherSpeed = LAUNCHER_SPEED_MIN;
      }

      if (launcherRunning) {
        applyLauncherSpeed();
      }
    }

    sendBoth("Launcher PWM=" + String(launcherSpeed));
    return;
  }

  // Pengatur sudut.
  if (command == 'Z') {
    startAngleHoming();
    return;
  }

  if (command == 'U') {
    jogLauncherAngle(ANGLE_JOG_DEG);
    return;
  }

  if (command == 'D') {
    jogLauncherAngle(-ANGLE_JOG_DEG);
    return;
  }

  if (command == 'C') {
    angleStopImmediate();
    sendBoth("Gerakan sudut dihentikan.");
    return;
  }

  if (command == 'V') {
    reportAngleStatus();
    return;
  }

  if (command == 'X') {
    emergencyStop();
    return;
  }

  const String validMotionCommands = "FBLRGIHJQES";

  if (validMotionCommands.indexOf(command) >= 0) {
    lastMotionCommand = command;
    executeMotion(command);

    Serial.printf(
        "Motion=%c, drive PWM=%d\n",
        command,
        driveSpeed
    );
  }
}

// Menerima input Bluetooth/Serial dan mendukung A30, A45.5, dan seterusnya.
void processInputByte(char input) {
  // Saat sedang membaca angka setelah huruf A.
  if (waitingAngleValue) {
    if ((input >= '0' && input <= '9') ||
        input == '.' || input == '-') {
      if (angleValueLength < sizeof(angleValueBuffer) - 1) {
        angleValueBuffer[angleValueLength++] = input;
        angleValueBuffer[angleValueLength] = '\0';
      }

      angleLastInputTime = millis();
      return;
    }

    // Enter, #, ;, spasi: selesaikan nilai sudut.
    if (input == '\r' || input == '\n' ||
        input == '#' || input == ';' || input == ' ') {
      finishAngleInput();
      return;
    }

    // Jika karakter perintah lain datang, selesaikan A terlebih dahulu.
    finishAngleInput();
  }

  char upper = toupper(static_cast<unsigned char>(input));

  if (upper == 'A') {
    waitingAngleValue = true;
    angleValueLength = 0;
    angleValueBuffer[0] = '\0';
    angleLastInputTime = millis();
    return;
  }

  processCommand(input);
}

// ============================================================
// SETUP DRIVER BTS7960
// ============================================================
bool setupMotor(const MotorPins &motor) {
  pinMode(motor.en, OUTPUT);
  digitalWrite(motor.en, LOW);

  bool rpwmOk = ledcAttach(motor.rpwm, PWM_FREQ, PWM_RESOLUTION);
  bool lpwmOk = ledcAttach(motor.lpwm, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(motor.rpwm, 0);
  ledcWrite(motor.lpwm, 0);

  return rpwmOk && lpwmOk;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  startBuzzer(BUZZER_BOOT_MS);

  bool pwmOK = true;
  pwmOK &= setupMotor(motorFL);
  pwmOK &= setupMotor(motorFR);
  pwmOK &= setupMotor(motorRL);
  pwmOK &= setupMotor(motorRR);
  pwmOK &= setupMotor(motorLauncher);

  driveStop();
  launcherStop();

  if (!pwmOK) {
    Serial.println("Gagal menginisialisasi salah satu kanal PWM.");

    while (true) {
      emergencyStop();
      delay(1000);
    }
  }

  // Limit switch bawah memakai pull-up internal ESP32.
  // Wiring disarankan: GPIO2 -> COM, GND -> NC.
  pinMode(ANGLE_MIN_LIMIT_PIN, INPUT_PULLUP);

  angleStepper.setPinsInverted(
      ANGLE_DIRECTION_INVERTED,
      false,
      false
  );
  // TB6600 lebih aman diberi pulsa STEP agak lebar.
  angleStepper.setMinPulseWidth(10);
  restoreNormalAngleMotionSettings();
  angleStepper.setCurrentPosition(0);
  angleStepper.moveTo(0);

  SerialBT.begin("OmniBot-ESP32");
  Serial2.begin(SERVO_UART_BAUD, SERIAL_8N1, SERVO_UART_RX_PIN, SERVO_UART_TX_PIN);

  Serial.println("Bluetooth aktif: OmniBot-ESP32");
  Serial.println("Gerak: F B L R G I H J Q E S");
  Serial.println("Speed roda: 0..9, +, -");
  Serial.println("Pelontar: K, M, P0..P9, [, ]");
  Serial.println("Sudut: Z home 1 switch, A0..A43, U naik, D turun, C stop, V status");
  Serial.println("ESPServo: Y conveyor ON, W ambil 1x, N stop+netral, T/O manual, ? status");
  Serial.println("Emergency stop: X");

  // Tunggu sebentar agar pembacaan limit switch stabil setelah ESP32 menyala.
  delay(250);

  // Otomatis homing saat pertama kali ESP32 dinyalakan/reset.
  // Proses homing dijalankan secara non-blocking oleh serviceAngleMotor()
  // di dalam loop(), sehingga Bluetooth dan fungsi lain tetap dapat diproses.
  Serial.println("Auto homing: mencari posisi 0 derajat...");
  startAngleHoming();

  lastCommandTime = millis();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  serviceBuzzer();
  serviceBluetoothBuzzer();
  serviceServoUart();

  // run() harus dipanggil sesering mungkin.
  serviceAngleMotor();

  while (SerialBT.available()) {
    processInputByte(static_cast<char>(SerialBT.read()));
    serviceAngleMotor();
    serviceServoUart();
  }

  while (Serial.available()) {
    processInputByte(static_cast<char>(Serial.read()));
    serviceAngleMotor();
    serviceServoUart();
  }

  // Menyelesaikan A30 yang dikirim tanpa Enter/# setelah timeout singkat.
  if (waitingAngleValue &&
      angleValueLength > 0 &&
      millis() - angleLastInputTime >= ANGLE_INPUT_TIMEOUT_MS) {
    finishAngleInput();
  }

  if (ENABLE_COMMAND_TIMEOUT &&
      lastMotionCommand != 'S' &&
      millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    driveStop();
    Serial.println("Failsafe: timeout, roda berhenti.");
  }

  // Jangan memakai delay beberapa milidetik karena akan membatasi
  // frekuensi pulsa STEP. yield() tetap memberi waktu pada sistem ESP32.
  yield();
}
