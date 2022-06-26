#include "MPU9250.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <obniz.h>
#include <TinyGPSPlus.h>

const float mC = 261.626; // ド
const float mD = 293.665; // レ
const float mE = 329.628; // ミ
const float mF = 349.228; // ファ
const float mG = 391.995; // ソ
const float mA = 440.000; // ラ 
const float mB = 493.883; // シ
const float nn = 0.0;

const float beep_wakeup[7] = {mE * 4, mA * 4, mB * 4, mA * 4, mE * 2, mE * 4, mB * 4}; // 起動音
const float beep_start[3] = {mC * 2, mD * 2, mE * 2};
const float beep_end[3] = {mE * 2, mD * 2, mC * 2};
const float beep_error[5] = {mE * 4, mE * 4, mE * 4, mE * 4, mE * 4};

MPU9250 mpu;
TinyGPSPlus gps;
HardwareSerial hs(2);

const uint8_t pin_sda        = 21;
const uint8_t pin_scl        = 22;
const uint8_t pin_button     = 35;
const uint8_t pin_led        = 2;
const uint8_t pin_heat       = 15;
const uint8_t pin_speaker    = 12;
const uint8_t pin_sd_miso    = 19;
const uint8_t pin_sd_mosi    = 23;
const uint8_t pin_sd_sclk    = 18;
const uint8_t pin_sd_cs      = 5;
const uint8_t pin_motor_A[3] = {4, 13, 25};  // AIN1, AIN2, PWMA
const uint8_t pin_motor_B[3] = {14, 27, 26}; // BIN1, BIN2, PWMB
const uint8_t pin_gps_tx     = 16;
const uint8_t pin_gps_rx     = 17;

const int CHANNEL_A = 0; // PWMA
const int CHANNEL_B = 1; // PWMB
const int CHANNEL_C = 2; // Speaker
 
const int LEDC_TIMER_8_BIT    = 8;
const int LEDC_TIMER_13_BIT   = 13;
const int LEDC_BASE_FREQ_490  = 490;
const int LEDC_BASE_FREQ_5000 = 490;

volatile byte led_state = LOW;
volatile long interrupt_prev_ms = millis();

struct SensorVal {
  float roll;
  float pitch;
  float yaw;
  float lat;
  float lng;
} sensorVal;

/** CanSatの状態遷移用の列挙型 */
enum {
  ST_STAND_BY = 0, // 待機
  ST_DRIVE,        // 目標地点へ走行
  ST_GOAL,         // 目標地点に到着
};

/** CanSatの状態遷移ステータス */
volatile int state = ST_STAND_BY;

/**
 * setup関数
 * 最初に1回だけ実行される
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Hello 100kinsat!");
  delay(2000);

  // 初期化処理
  obniz_init();
  pin_init();
  sd_init();
  mpu_init();
  gps_init();

  delay(2000);
  startUpdateMPUValTask();
  startUpdateGPSValTask();
  startSendObnizTask();

  beep(beep_wakeup, sizeof(beep_wakeup) / sizeof(float), 150);
}

/**
 * loop関数
 * 繰り返し実行される
 */
void loop() {
  switch (state)
  {
  case ST_STAND_BY:
    Serial.println("*** ST_STAND_BY ***");
    stand_by();
    break;

  case ST_DRIVE:
    Serial.println("*** ST_DRIVE ***");
    drive();
    break;

  case ST_GOAL:
    Serial.println("*** ST_GOAL ***");
    goal();
    break;
  
  default:
    break;
  }
  delay(200);
}

/** ボタンの割り込み関数 */
void IRAM_ATTR onButton() {
  if (millis() > interrupt_prev_ms + 500) { // チャタリング防止
    led_state = !led_state;
    state = (state + 1) % 3;
    interrupt_prev_ms = millis();
  }
}

/**
 * マルチタスクで実行する関数
 * 9軸センサの値の更新
 */
TaskHandle_t updateMPUValTaskHandle;
void updateMPUValTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
        prev_ms = millis();
      }
      sensorVal.roll = mpu.getRoll();
      sensorVal.pitch = mpu.getPitch();
      sensorVal.yaw = mpu.getYaw();
    }
    delay(10);
  }
}

/**
 * マルチタスクで実行する関数（updateMPUValTask）の開始
 */
void startUpdateMPUValTask() {
  xTaskCreatePinnedToCore(
    updateMPUValTask,
    "updateMPUValTask",
    8192,
    NULL,
    1,
    &updateMPUValTaskHandle,
    APP_CPU_NUM
  );
}

/**
 * マルチタスクで実行する関数
 * GPSセンサの値の更新
 */
TaskHandle_t updateGPSValTaskHandle;
void updateGPSValTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    while (hs.available() > 0) {
      char c = hs.read();
      gps.encode(c);
      if (gps.location.isUpdated()) {
        sensorVal.lat = gps.location.lat();
        sensorVal.lng = gps.location.lng();
      }
    }
    delay(5000);
  }
}

/**
 * マルチタスクで実行する関数（updateGPSValTask）の開始
 */
void startUpdateGPSValTask() {
  xTaskCreatePinnedToCore(
    updateGPSValTask,
    "updateGPSValTask",
    8192,
    NULL,
    1,
    &updateGPSValTaskHandle,
    APP_CPU_NUM
  );
}

/**
 * マルチタスクで実行する関数
 * CanSat -> JavaScript へのデータ送信
 */
TaskHandle_t sendObnizTaskHandle;
void sendObnizTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    char message[512]; // JavaScriptへ送信するメッセージ用
    char roll_buf[16];
    char pitch_buf[16];
    char yaw_buf[16];
    char lat_buf[16];
    char lng_buf[16];

    if (obniz.isOnline()) {
      dtostrf(sensorVal.roll, -1, 2, (char*)roll_buf);
      dtostrf(sensorVal.pitch, -1, 2, (char*)pitch_buf);
      dtostrf(sensorVal.yaw, -1, 2, (char*)yaw_buf);
      dtostrf(sensorVal.lat, -1, 6, (char*)lat_buf);
      dtostrf(sensorVal.lng, -1, 6, (char*)lng_buf);
      sprintf(message, "%d,%s,%s,%s,%s,%s",
        state,
        roll_buf, pitch_buf, yaw_buf,
        lat_buf, lng_buf);
      obniz.commandSend((uint8_t*)message, strlen(message));
    }
    delay(500);
  }
}

/**
 * マルチタスクで実行する関数（sendObnizTask）の開始
 */
void startSendObnizTask() {
  xTaskCreatePinnedToCore(
    sendObnizTask,
    "sendObnizTask",
    8192,
    NULL,
    1,
    &sendObnizTaskHandle,
    APP_CPU_NUM
  );
}

/** 待機状態 */
void stand_by() {
  digitalWrite(pin_led, led_state);
}

/** 目標地点へ走行 */
void drive() {
  forward(255);
  delay(5000);
  stop();
  delay(2000);
  back(100);
  delay(5000);
  stop();
  delay(2000);
}

/** 目標地点に到着 */
void goal() {
}

/** ObnizOSの初期化処理 */
void obniz_init() {
  obniz.start(NULL); // 引数にNULLを渡すとObnizOSのログがシリアルモニタに表示されなくなる
  // IOの管理をobnizOSから外す
  obniz.pinReserve(pin_sda);
  obniz.pinReserve(pin_scl);
  obniz.pinReserve(pin_button);
  obniz.pinReserve(pin_led);
  obniz.pinReserve(pin_heat);
  obniz.pinReserve(pin_speaker);
  obniz.pinReserve(pin_sd_miso);
  obniz.pinReserve(pin_sd_mosi);
  obniz.pinReserve(pin_sd_sclk);
  obniz.pinReserve(pin_sd_cs);
  for (int i = 0; i < 3; i++) {
    obniz.pinReserve(pin_motor_A[i]);
    obniz.pinReserve(pin_motor_B[i]);
  }
  obniz.pinReserve(pin_gps_tx);
  obniz.pinReserve(pin_gps_rx);
}

/** GPIOの初期化処理 */
void pin_init() {
  pinMode(pin_button, INPUT);
  attachInterrupt(pin_button, onButton, FALLING);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_heat, OUTPUT);
  digitalWrite(pin_heat, LOW); // 電熱線のピンはLOWにしておく
  pinMode(pin_speaker, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(pin_motor_A[i], OUTPUT);
    pinMode(pin_motor_B[i], OUTPUT);
  }
  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ_490, LEDC_TIMER_8_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ_490, LEDC_TIMER_8_BIT);
  ledcAttachPin(pin_motor_A[2], CHANNEL_A);
  ledcAttachPin(pin_motor_B[2], CHANNEL_B);
}

/** SDカードの初期化処理 */
void sd_init() {
  if (!SD.begin()) {
    Serial.println("Card mount failed.");
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached.");
    // 初期化に失敗したらエラー音を鳴らす
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  writeFile(SD, "/100kinsat.txt", "Hello 100kinSAT!!!");
}

/** 9軸センサの初期化処理 */
void mpu_init() {
  Wire.begin();
  delay(2000);

  // if (!mpu.setup(0x69)) { // サイはI2Cアドレスが違う
  if (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed.");
    // 初期化に失敗したらエラー音を鳴らす
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }
  mpu.setMagneticDeclination(-7.49); // 磁気偏角の設定（府中駅: -7.49）
  mpu.selectFilter(QuatFilterSel::MADGWICK); // フィルターの設定
  mpu.setFilterIterations(10);

  mpu.verbose(true);

  // 加速度/ジャイロセンサのキャリブレーション
  // キャリブレーション中はCanSatを平らな地面で静止させておく
  beep(beep_start, sizeof(beep_start) / sizeof(float), 150);
  delay(500);
  mpu.calibrateAccelGyro();
  beep(beep_end, sizeof(beep_end) / sizeof(float), 150);

  delay(1000);

  // 地磁気センサのキャリブレーション
  // キャリブレーション中はCanSatをぐるぐる回転させる
  beep(beep_start, sizeof(beep_start) / sizeof(float), 150);
  delay(500);
  mpu.calibrateMag();
  beep(beep_end, sizeof(beep_end) / sizeof(float), 150);

  mpu.verbose(false);
}

/** GPSセンサの初期化処理 */
void gps_init() {
  hs.begin(9600);
}

/** 前進 */
void forward(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CCW，反時計回り）
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(pin_motor_B[1], LOW);
  digitalWrite(pin_motor_B[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 後退 */
void back(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CW，時計回り）
  digitalWrite(pin_motor_A[1], LOW);
  digitalWrite(pin_motor_A[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 停止 */
void stop() {
  // 左モータ停止
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], LOW);
  ledcWrite(CHANNEL_A, HIGH);

  // 右モータ停止
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], LOW);
  ledcWrite(CHANNEL_B, HIGH);
}

/** SDカードに新規書き込みする */
 void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

/** SDカードに追記する */
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

/** スピーカーから音を鳴らす */
void beep(const float *mm, int m_size, int t_ms) {
  for (int i = 0; i < m_size; i++) {
    tone(pin_speaker, mm[i], t_ms);
  }
  noTone(pin_speaker);
}

void tone(int pin, int freq, int t_ms) {
  ledcSetup(CHANNEL_C, LEDC_BASE_FREQ_5000, LEDC_TIMER_13_BIT);
  ledcAttachPin(pin, CHANNEL_C);
  ledcWriteTone(CHANNEL_C, freq);

  delay(t_ms);
}

void noTone(int pin) {
  ledcWriteTone(CHANNEL_C, 0.0);
}
