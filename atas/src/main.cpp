#include <Arduino.h>
#include <ESP32Servo.h>

struct LineMuxPins {
  uint8_t s0;
  uint8_t s1;
  uint8_t s2;
  uint8_t s3;
  uint8_t sig;
};

constexpr uint8_t LINE_SENSOR_COUNT = 16;
constexpr uint16_t LINE_CALIBRATION_SAMPLES = 64;
constexpr uint16_t LINE_READ_SAMPLES = 8;
constexpr uint16_t LINE_SETTLE_DELAY_US = 12;
constexpr uint32_t LINE_REPORT_INTERVAL_MS = 500;

const LineMuxPins LINE_RIGHT = {18, 5, 17, 16, 4};
const LineMuxPins LINE_LEFT  = {15, 2, 22, 23, 12};

uint16_t lineRightWhite[LINE_SENSOR_COUNT];
uint16_t lineRightGreen[LINE_SENSOR_COUNT];
uint16_t lineRightThreshold[LINE_SENSOR_COUNT];
uint16_t lineLeftWhite[LINE_SENSOR_COUNT];
uint16_t lineLeftGreen[LINE_SENSOR_COUNT];
uint16_t lineLeftThreshold[LINE_SENSOR_COUNT];
uint16_t lineRightBinary = 0;
uint16_t lineLeftBinary = 0;
uint32_t lastLineReportMs = 0;
bool lineRightCalibrated = false;
bool lineLeftCalibrated = false;
bool lineRightWhiteCaptured = false;
bool lineLeftWhiteCaptured = false;


// =========================================================
// KOMUNIKASI UART DARI SHUTTLEBOT
// =========================================================
// Wiring silang:
//   ShuttleBot TX -> ESPServo RX2 GPIO16
//   ShuttleBot RX -> ESPServo TX2 GPIO17
//   GND ShuttleBot -> GND ESPServo
const uint8_t PIN_UART_RX = 3;
const uint8_t PIN_UART_TX = 1;
const uint32_t UART_BAUD = 115200;

// =========================================================
// PIN SERVO
// =========================================================
const uint8_t PIN_SERVO_ARM    = 27;
const uint8_t PIN_SERVO_GRIPER = 26;

// =========================================================
// PIN DRIVER MOTOR L298N
// =========================================================
const uint8_t PIN_ENA = 25;
const uint8_t PIN_IN1 = 32;
const uint8_t PIN_IN2 = 33;

// =========================================================
// POSISI SERVO
// =========================================================
const int ARM_NAIK     = 150;
const int ARM_TURUN    = 0;

const int GRIPER_BUKA  = 5;
const int GRIPER_TUTUP = 110;

// =========================================================
// PENGATURAN MOTOR DC
// =========================================================
const int KECEPATAN_MOTOR = 185;

const int MOTOR_PWM_FREQ = 490;
const int MOTOR_PWM_RES  = 8;

// Semakin besar nilainya, gerakan servo semakin lambat.
const int SERVO_SPEED_MS = 0;

// Loncat sudut lebih besar = gerak lebih cepat.
const int SERVO_STEP_DEG = 20;

// =========================================================
// OBJEK SERVO
// =========================================================
Servo servoArm;
Servo servoGriper;

int posisiArm    = ARM_TURUN;
int posisiGriper = GRIPER_BUKA;

// =========================================================
// FUNGSI MOTOR DC
// =========================================================
void motorStop() {
  ledcWrite(PIN_ENA, 0);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

void motorMaju(int kecepatan) {
  kecepatan = constrain(kecepatan, 0, 255);

  if (kecepatan == 0) {
    motorStop();
    return;
  }

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);

  ledcWrite(PIN_ENA, kecepatan);
}

// =========================================================
// FUNGSI GERAK SERVO PER DERAJAT
// =========================================================
void gerakkanServo(
  Servo &servo,
  int &posisiSekarang,
  int posisiTujuan,
  int kecepatanMs
) {
  posisiTujuan = constrain(posisiTujuan, 0, 180);

  if (posisiSekarang < posisiTujuan) {
    for (
      int posisi = posisiSekarang;
      posisi <= posisiTujuan;
      posisi += SERVO_STEP_DEG
    ) {
      servo.write(posisi);
      delay(kecepatanMs);
    }
  } else if (posisiSekarang > posisiTujuan) {
    for (
      int posisi = posisiSekarang;
      posisi >= posisiTujuan;
      posisi -= SERVO_STEP_DEG
    ) {
      servo.write(posisi);
      delay(kecepatanMs);
    }
  }

  posisiSekarang = posisiTujuan;
}

// =========================================================
// URUTAN GERAKAN
// =========================================================
void jalankanUrutan() {
  Serial.println("1. Arm naik");

  gerakkanServo(
    servoArm,
    posisiArm,
    ARM_NAIK,
    SERVO_SPEED_MS
  );

  delay(150);

  Serial.println("2. Gripper menutup");

  gerakkanServo(
    servoGriper,
    posisiGriper,
    GRIPER_TUTUP,
    SERVO_SPEED_MS
  );

  delay(150);

  Serial.println("3. Arm turun");

  gerakkanServo(
    servoArm,
    posisiArm,
    ARM_TURUN,
    SERVO_SPEED_MS
  );

  delay(150);

  Serial.println("4. Gripper membuka");

  gerakkanServo(
    servoGriper,
    posisiGriper,
    GRIPER_BUKA,
    SERVO_SPEED_MS
  );

  delay(150);

  Serial.println("Urutan selesai");
}

void kirimStatus(const String &pesan) {
  Serial.println(pesan);
  Serial2.println(pesan);
}

void setupLineMux(const LineMuxPins &pins) {
  pinMode(pins.s0, OUTPUT);
  pinMode(pins.s1, OUTPUT);
  pinMode(pins.s2, OUTPUT);
  pinMode(pins.s3, OUTPUT);
  pinMode(pins.sig, INPUT);
}

void selectLineChannel(const LineMuxPins &pins, uint8_t channel) {
  digitalWrite(pins.s0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(pins.s1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(pins.s2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(pins.s3, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(LINE_SETTLE_DELAY_US);
}

uint16_t readLineChannelRaw(const LineMuxPins &pins, uint8_t channel) {
  uint32_t total = 0;
  selectLineChannel(pins, channel);
  for (uint16_t sample = 0; sample < LINE_READ_SAMPLES; ++sample) {
    total += analogRead(pins.sig);
  }
  return total / LINE_READ_SAMPLES;
}

void readLineCalibrationColor(const LineMuxPins &pins, uint16_t *values) {
  uint32_t totals[LINE_SENSOR_COUNT] = {};

  for (uint16_t sample = 0; sample < LINE_CALIBRATION_SAMPLES; ++sample) {
    for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
      totals[channel] += readLineChannelRaw(pins, channel);
    }
    delay(1);
  }

  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    values[channel] = totals[channel] / LINE_CALIBRATION_SAMPLES;
  }
}

void saveLineWhite(const LineMuxPins &pins, uint16_t *white, const char *label) {
  readLineCalibrationColor(pins, white);
  kirimStatus(String("Kalibrasi putih ") + label + " tersimpan");
}

void saveLineGreen(
    const LineMuxPins &pins,
    const uint16_t *white,
    uint16_t *green,
    uint16_t *thresholds,
    const char *label) {
  readLineCalibrationColor(pins, green);

  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    thresholds[channel] = (static_cast<uint32_t>(white[channel]) + green[channel]) / 2;
  }

  kirimStatus(String("Kalibrasi hijau ") + label + " selesai, threshold tersimpan");
}

uint16_t readLineSideBinary(
    const LineMuxPins &pins,
    const uint16_t *white,
    const uint16_t *green,
    const uint16_t *thresholds) {
  uint16_t bits = 0;
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    uint16_t value = readLineChannelRaw(pins, channel);
    bool greenIsHigher = green[channel] > white[channel];
    bool active = greenIsHigher ? value < thresholds[channel] : value > thresholds[channel];

    if (active) {
      bits |= (1u << channel);
    }
  }
  return bits;
}

String bitsToBinary16(uint16_t value) {
  String text;
  text.reserve(16);
  for (int8_t bit = 15; bit >= 0; --bit) {
    text += ((value >> bit) & 1u) ? '1' : '0';
  }
  return text;
}

void saveLineWhiteAll() {
  setupLineMux(LINE_RIGHT);
  setupLineMux(LINE_LEFT);
  saveLineWhite(LINE_RIGHT, lineRightWhite, "kanan");
  saveLineWhite(LINE_LEFT, lineLeftWhite, "kiri");
  lineRightWhiteCaptured = true;
  lineLeftWhiteCaptured = true;
  lineRightCalibrated = false;
  lineLeftCalibrated = false;
  kirimStatus("Putih tersimpan. Taruh sensor di hijau lalu kirim #");
}

void saveLineGreenAll() {
  if (!lineRightWhiteCaptured || !lineLeftWhiteCaptured) {
    kirimStatus("Kalibrasi putih dulu. Kirim @");
    return;
  }

  saveLineGreen(LINE_RIGHT, lineRightWhite, lineRightGreen, lineRightThreshold, "kanan");
  saveLineGreen(LINE_LEFT, lineLeftWhite, lineLeftGreen, lineLeftThreshold, "kiri");
  lineRightCalibrated = true;
  lineLeftCalibrated = true;
}

void saveLineWhiteRight() {
  setupLineMux(LINE_RIGHT);
  saveLineWhite(LINE_RIGHT, lineRightWhite, "kanan");
  lineRightWhiteCaptured = true;
  lineRightCalibrated = false;
  kirimStatus("Putih kanan tersimpan. Taruh kanan di hijau lalu kirim .");
}

void saveLineGreenRight() {
  if (!lineRightWhiteCaptured) {
    kirimStatus("Kalibrasi putih kanan dulu. Kirim >");
    return;
  }

  saveLineGreen(LINE_RIGHT, lineRightWhite, lineRightGreen, lineRightThreshold, "kanan");
  lineRightCalibrated = true;
}

void saveLineWhiteLeft() {
  setupLineMux(LINE_LEFT);
  saveLineWhite(LINE_LEFT, lineLeftWhite, "kiri");
  lineLeftWhiteCaptured = true;
  lineLeftCalibrated = false;
  kirimStatus("Putih kiri tersimpan. Taruh kiri di hijau lalu kirim ,");
}

void saveLineGreenLeft() {
  if (!lineLeftWhiteCaptured) {
    kirimStatus("Kalibrasi putih kiri dulu. Kirim <");
    return;
  }

  saveLineGreen(LINE_LEFT, lineLeftWhite, lineLeftGreen, lineLeftThreshold, "kiri");
  lineLeftCalibrated = true;
}

void updateLineSensors() {
  if (!lineRightCalibrated || !lineLeftCalibrated) {
    return;
  }

  lineRightBinary = readLineSideBinary(LINE_RIGHT, lineRightWhite, lineRightGreen, lineRightThreshold);
  lineLeftBinary = readLineSideBinary(LINE_LEFT, lineLeftWhite, lineLeftGreen, lineLeftThreshold);
}

void reportLineSensors() {
  if (!lineRightCalibrated || !lineLeftCalibrated) {
    kirimStatus("LINE belum lengkap. @ putih semua, # hijau semua, >/. kanan, </, kiri");
    return;
  }

  kirimStatus("LINE kanan=" + bitsToBinary16(lineRightBinary) +
              " kiri=" + bitsToBinary16(lineLeftBinary));
}

void handleLineCommand(char perintah) {
  if (perintah == '@') {
    saveLineWhiteAll();
    return;
  }

  if (perintah == '#') {
    saveLineGreenAll();
    updateLineSensors();
    reportLineSensors();
    return;
  }

  if (perintah == '>') {
    saveLineWhiteRight();
    return;
  }

  if (perintah == '.') {
    saveLineGreenRight();
    updateLineSensors();
    reportLineSensors();
    return;
  }

  if (perintah == '<') {
    saveLineWhiteLeft();
    return;
  }

  if (perintah == ',') {
    saveLineGreenLeft();
    updateLineSensors();
    reportLineSensors();
    return;
  }

  if (perintah == '*') {
    updateLineSensors();
    reportLineSensors();
  }
}

bool isLineCommand(char perintah) {
  return perintah == '@' || perintah == '#' || perintah == '*' ||
         perintah == '>' || perintah == '.' ||
         perintah == '<' || perintah == ',';
}

void serviceLineSensors() {
  if (!lineRightCalibrated || !lineLeftCalibrated) {
    return;
  }

  updateLineSensors();

  if (millis() - lastLineReportMs >= LINE_REPORT_INTERVAL_MS) {
    lastLineReportMs = millis();
    reportLineSensors();
  }
}

void prosesPerintah(char perintah) {
  if (perintah == '\r' || perintah == '\n' || perintah == ' ' || perintah == '\t') {
    return;
  }

  perintah = toupper(static_cast<unsigned char>(perintah));

  if (isLineCommand(perintah)) {
    handleLineCommand(perintah);
    return;
  }

  if (perintah == 'Y') {
    motorMaju(KECEPATAN_MOTOR);
    kirimStatus("ESPServo: conveyor ON");
    return;
  }

  if (perintah == 'W') {
    kirimStatus("ESPServo: ambil mulai");
    jalankanUrutan();
    kirimStatus("ESPServo: ambil selesai");
    return;
  }

  if (perintah == 'T') {
    gerakkanServo(servoArm, posisiArm, ARM_NAIK, SERVO_SPEED_MS);
    kirimStatus("ESPServo: arm naik");
    return;
  }

  if (perintah == 'O') {
    gerakkanServo(servoGriper, posisiGriper, GRIPER_TUTUP, SERVO_SPEED_MS);
    kirimStatus("ESPServo: gripper tutup");
    return;
  }

  if (perintah == 'N') {
    motorStop();
    gerakkanServo(servoArm, posisiArm, ARM_TURUN, SERVO_SPEED_MS);
    gerakkanServo(servoGriper, posisiGriper, GRIPER_BUKA, SERVO_SPEED_MS);
    kirimStatus("ESPServo: netral");
    return;
  }

  if (perintah == '?') {
    kirimStatus("ESPServo OK. Line: @ putih semua, # hijau semua, >/. kanan, </, kiri, * baca");
  }
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  delay(500);

  Serial.println();
  Serial.println("Memulai sistem...");

  // =======================================================
  // SETUP SERVO
  // Servo di-attach sebelum motor PWM.
  // =======================================================
  servoArm.setPeriodHertz(50);
  servoGriper.setPeriodHertz(50);

  int channelArm = servoArm.attach(
    PIN_SERVO_ARM,
    500,
    2500
  );

  int channelGriper = servoGriper.attach(
    PIN_SERVO_GRIPER,
    500,
    2500
  );

  Serial.print("Channel servo arm: ");
  Serial.println(channelArm);

  Serial.print("Channel servo gripper: ");
  Serial.println(channelGriper);

  Serial.print("Servo Arm attached: ");
  Serial.println(
    servoArm.attached() ? "YA" : "TIDAK"
  );

  Serial.print("Servo Gripper attached: ");
  Serial.println(
    servoGriper.attached() ? "YA" : "TIDAK"
  );

  setupLineMux(LINE_RIGHT);
  setupLineMux(LINE_LEFT);
  kirimStatus("Sensor garis siap. Android: @ putih semua, # hijau semua, >/. kanan, </, kiri");

  // Posisi awal saat ESP32 menyala.
  servoArm.write(ARM_TURUN);
  servoGriper.write(GRIPER_BUKA);

  posisiArm    = ARM_TURUN;
  posisiGriper = GRIPER_BUKA;

  // Beri waktu servo menuju posisi awal.
  delay(1000);

  // =======================================================
  // SETUP DRIVER MOTOR
  // Arduino-ESP32 Core 3.x: PWM pakai pin langsung.
  // =======================================================
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  bool pwmOk = ledcAttach(PIN_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

  Serial.print("PWM motor attach: ");
  Serial.println(pwmOk ? "OK" : "GAGAL");

  motorStop();

  Serial.println("Motor berhenti saat awal");
  delay(2000);

  Serial2.println("ESPServo siap");
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  serviceLineSensors();

  while (Serial2.available()) {
    prosesPerintah(static_cast<char>(Serial2.read()));
  }

  while (Serial.available()) {
    prosesPerintah(static_cast<char>(Serial.read()));
  }
}
