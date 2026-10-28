// ============================================================
//  Робот MGbot ЙоТик: приём команд пульта по ESP-NOW
//  НЕ ПРОВЕРЕНО НА ЖЕЛЕЗЕ: первый запуск — колёса в воздухе.
//
//  Библиотека MGB_MDYN2 лежит в lib/MGB_MDYN2/, ставить её не нужно.
//  Скорость бортов меняется по рампе (RAMP_PCT_PER_S); при потере связи — мгновенный стоп.
//  Приём broadcast-пакетов: робот реагирует только на свой ROBOT_ID и на один пульт.
//  Wi-Fi-клиента и Telegram здесь НЕТ: WiFi.begin() не вызываем,
//  поэтому канал радио остаётся стандартным и совпадает с пультом.
// ============================================================
#include <Arduino.h>   // в .cpp (в отличие от .ino) подключается явно
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <MGB_MDYN2.h>   // из lib/MGB_MDYN2/, PlatformIO находит сам

// ---- 1. НАСТРОЙКИ ------------------------------------------
// Номер робота задаётся при сборке: окружения robot1..robot3 в platformio.ini
// (build_flags = -DROBOT_ID=N). #ifndef/#error — защита от сборки без номера.
#ifndef ROBOT_ID
#error "Не задан ROBOT_ID: собирайте окружения robot1, robot2 или robot3"
#endif
const uint8_t  PIN_LED         = 18;     // LED3 на плате (как в исходном скетче)
const uint32_t FAILSAFE_MS     = 300;    // нет пакетов дольше — моторы СТОП
const uint32_t CONTROL_PERIOD_MS = 40;   // 25 раз в секунду обновляем моторы
const float    SPEED           = 35;     // % мощности при нажатой кнопке; подобрать на роботе
const float    RAMP_PCT_PER_S  = 200;    // рампа: на сколько % в секунду может меняться мощность
                                         // (200 → разгон 0→35 % за ~0,2 с, реверс 35→-35 % за ~0,35 с)
const int      TURN_SIGN       = +1;     // если «вправо» поворачивает влево — поставить -1

// ---- 2. ФОРМАТ ПАКЕТА (копия из пульта, менять только вместе) ----
typedef struct __attribute__((packed)) {
  uint8_t magic, seq, robotId, buttons;
  int8_t  axisX, axisY;
} RemotePacket;

const uint8_t PKT_MAGIC = 0xA5;
const uint8_t BTN_UP = 1 << 0, BTN_DOWN = 1 << 1, BTN_LEFT = 1 << 2, BTN_RIGHT = 1 << 3;

// ---- 3. ПЛАТА МОТОРОВ (инициализация из исходного скетча MGbot) ----
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x70);
bool TCA9548A;

int setI2Cmodule(void) {
  int error;
  Wire.beginTransmission(0x70);
  delay(100);
  error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("\nNo I2C Module / Broken Module\n");
    Serial.print("Error #");
    Serial.print(error);
    Serial.print("\n");
    return error;
  }
  Wire.beginTransmission(0x70);
  delay(100);
  Wire.write(0x06 | 0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x60);  // опрос подключенного к выводу MGB-BUZ1
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("\nOld Type Module");
    TCA9548A = false;
    return error;
  }
  Wire.beginTransmission(0x70);
  delay(100);
  Wire.write(0x01 << 0x06);
  Wire.endTransmission();
  Wire.beginTransmission(0x60);  // опрос подключенного к выводу MGB-BUZ1
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("\nNew Type Module");
    TCA9548A = true;
    return error;
  }
  Serial.print("\nError #");
  Serial.print(error);
  return error;
}

// Скорости бортов в процентах, -100..100.
// В библиотеке motor_setpower(мотор, мощность, invert): мощность ЗНАКОВАЯ,
// invert просто меняет знак. Моторы стоят зеркально, поэтому у мотора 2
// invert = true: тогда «положительная» скорость означает «вперёд» для обоих.
// (В исходном скетче «вперёд» = (1, 30, false) и (2, 30, true) — то же самое.)
void drive(float left, float right) {
  mdyn2.motor_setpower(1, left,  false);   // предположение: мотор 1 — ЛЕВЫЙ борт
  mdyn2.motor_setpower(2, right, true);
}

// Рампа: приближает текущее значение cur к целевому target,
// но не больше чем на maxStep за один вызов.
float rampTo(float cur, float target, float maxStep) {
  float d = target - cur;           // сколько осталось пройти
  if (d >  maxStep) d =  maxStep;   // ограничиваем шаг сверху
  if (d < -maxStep) d = -maxStep;   // и снизу (торможение/реверс)
  return cur + d;
}

// ---- 4. ПРИЁМ: ОБЩИЕ ДАННЫЕ МЕЖДУ КОЛБЭКОМ И loop() --------
// Колбэк вызывается из задачи Wi-Fi, loop() — из другой задачи. Чтобы не прочитать
// структуру «наполовину обновлённой», доступ к ней обёрнут в критическую секцию.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
RemotePacket shared;
uint32_t lastRxMs = 0;
bool     everRx   = false;
uint8_t  owner[6] = {0};     // MAC пульта, который сейчас управляет роботом

// Общая обработка. src — MAC отправителя (пульта).
void handlePacket(const uint8_t *src, const uint8_t *data, int len) {
  if ((size_t)len != sizeof(RemotePacket)) return;   // чужой/битый пакет
  if (data[0] != PKT_MAGIC) return;
  RemotePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.robotId != ROBOT_ID) return;               // пакет для другого робота

  portENTER_CRITICAL(&mux);                          // вход в критическую секцию
  uint32_t now = millis();
  // Владелец свободен, если пакетов ещё не было или связь пропала дольше FAILSAFE_MS.
  bool ownerFree = !everRx || (now - lastRxMs >= FAILSAFE_MS);
  if (ownerFree) memcpy(owner, src, 6);              // первый заговоривший становится владельцем
  if (memcmp(owner, src, 6) == 0) {                  // memcmp == 0: адреса совпали
    shared = pkt;                                    // чужие пульты с тем же номером игнорируются
    lastRxMs = now;
    everRx = true;
  }
  portEXIT_CRITICAL(&mux);                           // выход
  // Моторами из колбэка НЕ управляем: он должен возвращаться быстро.
}

// Сигнатура колбэка приёма отличается между ядрами ESP32 2.x и 3.x,
// макрос ESP_ARDUINO_VERSION_MAJOR выбирает нужный вариант при компиляции.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handlePacket(info->src_addr, data, len);
}
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  handlePacket(mac, data, len);
}
#endif

void setup() {
  Serial.begin(115200);

  Wire.begin();                 // I2C: GPIO21 (SDA), GPIO22 (SCL)
  setI2Cmodule();
  mdyn2.begin();
  mdyn2.setPWMFreq(100);        // частота ШИМ, Гц
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);          // все каналы платы выключены
  }
  pinMode(PIN_LED, OUTPUT);

  // Номер робота: LED3 мигает ROBOT_ID раз (удобно проверить, какая прошивка в каком роботе).
  for (int i = 0; i < ROBOT_ID; i++) {
    digitalWrite(PIN_LED, HIGH); delay(200);
    digitalWrite(PIN_LED, LOW);  delay(250);
  }
  Serial.printf("\nROBOT_ID = %d\n", ROBOT_ID);

  WiFi.mode(WIFI_STA);
  Serial.print("\nMAC робота: ");          // (для справки: в broadcast-схеме MAC в пульт не вписывается)
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Ошибка esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);    // передаём САМУ функцию (без скобок)
}

void loop() {
  static uint32_t tCtl = 0;
  uint32_t now = millis();
  if (now - tCtl < CONTROL_PERIOD_MS) return;
  float dt = (now - tCtl) * 0.001f;   // секунд с прошлого цикла
  if (dt > 0.1f) dt = 0.1f;           // защита от огромного dt в самом первом цикле
  tCtl = now;

  static float curL = 0, curR = 0;    // фактические скорости бортов, %; static — значения живут между вызовами loop()

  // Снимок общих данных под защитой.
  RemotePacket p;
  uint32_t t;
  bool got;
  portENTER_CRITICAL(&mux);
  p = shared;  t = lastRxMs;  got = everRx;
  portEXIT_CRITICAL(&mux);

  // Разность беззнаковых чисел корректна даже при переполнении millis().
  bool linkOk = got && (millis() - t < FAILSAFE_MS);

  // Индикация связи на RGB-светодиоде 1 платы моторов: зелёный — есть, красный — нет.
  // Меняем только при смене состояния, чтобы не грузить I2C. Если цвета не те или
  // светодиодов нет — эти две строки можно удалить, на работу моторов это не влияет.
  static int8_t prevLink = -1;                       // -1: ещё ни разу не выставляли
  if ((int8_t)linkOk != prevLink) {
    prevLink = (int8_t)linkOk;
    if (linkOk) mdyn2.rgb_set(1, 0, 40, 0);
    else        mdyn2.rgb_set(1, 40, 0, 0);
  }

  if (!linkOk) {            // FAILSAFE: связи нет — стоим МГНОВЕННО, без рампы
    curL = 0; curR = 0;     // и сбрасываем рампу, чтобы потом разгон шёл с нуля
    drive(0, 0);
    return;
  }

  int y = ((p.buttons & BTN_UP)    ? 1 : 0) - ((p.buttons & BTN_DOWN) ? 1 : 0);   // вперёд − назад
  int x = ((p.buttons & BTN_RIGHT) ? 1 : 0) - ((p.buttons & BTN_LEFT) ? 1 : 0);   // вправо − влево

  // Смешивание для дифференциального привода. x = ±1, y = 0: разворот на месте.
  // constrain(v, lo, hi) зажимает значение в диапазон [lo, hi].
  float L = constrain((y + TURN_SIGN * x) * SPEED, -100.0f, 100.0f);   // целевые скорости бортов
  float R = constrain((y - TURN_SIGN * x) * SPEED, -100.0f, 100.0f);

  float step = RAMP_PCT_PER_S * dt;   // максимум изменения за этот цикл, %
  curL = rampTo(curL, L, step);
  curR = rampTo(curR, R, step);
  drive(curL, curR);
}
