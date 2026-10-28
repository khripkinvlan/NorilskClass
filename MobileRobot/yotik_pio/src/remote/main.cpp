// ============================================================
//  Пульт ДУ на ЙоТик 32B v2.0 → робот MGbot ЙоТик (ESP-NOW)
//  НЕ ПРОВЕРЕНО НА ЖЕЛЕЗЕ: сначала прошить и протестировать.
//
//  Подключение: каждая кнопка между пином и GND (нажата = LOW).
//    Вперёд  GPIO17 (J3.TX2)    Назад   GPIO16 (J3.RX2)
//    Влево   GPIO21 (J3.SDA)    Вправо  GPIO22 (J3.SCL)
//    LED3 на плате = GPIO18 (монтаж не нужен)
//  Пины сверены по документации ЙоТик 32B v2.0B; для плат v2.4 — сверить.
//  Библиотека MGbot на пульте НЕ нужна. MAC роботов не нужны: broadcast + номер робота.
// ============================================================
#include <Arduino.h>   // в .cpp (в отличие от .ino) подключается явно
#include <WiFi.h>      // нужен, чтобы включить радиомодуль в режиме STA
#include <esp_now.h>   // сам протокол ESP-NOW

// ---- 1. НАСТРОЙКИ ------------------------------------------
// Номер робота, которым управляем (наклейка 1..3 на роботе). Участник меняет только это.
const uint8_t TARGET_ROBOT = 1;
static_assert(TARGET_ROBOT >= 1 && TARGET_ROBOT <= 3, "TARGET_ROBOT должен быть 1..3");
// static_assert — проверка при КОМПИЛЯЦИИ: при неверном номере сборка остановится с этим сообщением.

// Широковещательный адрес: пакет принимают ВСЕ устройства в зоне, адресат определяется по robotId.
uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint8_t PIN_UP = 17, PIN_DOWN = 16, PIN_LEFT = 21, PIN_RIGHT = 22;
const uint8_t PIN_LED = 18;                 // LED3 на плате

const uint32_t SEND_PERIOD_MS = 33;         // ~30 пакетов в секунду
const uint32_t DEBOUNCE_MS    = 20;         // время «успокоения» контакта

// ---- 2. ФОРМАТ ПАКЕТА (должен совпадать с приёмником байт в байт) ----
// typedef struct {...} Имя;  — задаёт имя типа, дальше пишем просто RemotePacket.
// __attribute__((packed))    — расширение GCC: запрет вставки байтов-выравнивателей,
//                              поэтому размер и порядок полей одинаковы на обеих платах.
typedef struct __attribute__((packed)) {
  uint8_t magic;     // 0xA5: отсеивает чужие пакеты
  uint8_t seq;       // счётчик пакетов: по пропускам видно потери
  uint8_t robotId;   // для какого робота (1..3); остальные роботы пакет игнорируют
  uint8_t buttons;   // удерживаемые сейчас: биты BTN_*
  int8_t  axisX;     // -100..100, резерв под джойстик (пока 0)
  int8_t  axisY;
} RemotePacket;

const uint8_t PKT_MAGIC = 0xA5;
// Битовые маски: 1 << n — единица, сдвинутая на n разрядов влево.
const uint8_t BTN_UP = 1 << 0, BTN_DOWN = 1 << 1, BTN_LEFT = 1 << 2, BTN_RIGHT = 1 << 3;

// ---- 3. КНОПКА С АНТИДРЕБЕЗГОМ -------------------------------
struct Btn {
  uint8_t  pin;
  bool     state;   // отфильтрованное состояние (true = нажата)
  bool     raw;     // последнее «сырое» чтение
  uint32_t tRaw;    // когда сырое значение менялось в последний раз
};

Btn bUp = {PIN_UP, false, false, 0},     bDown = {PIN_DOWN, false, false, 0};
Btn bLeft = {PIN_LEFT, false, false, 0}, bRight = {PIN_RIGHT, false, false, 0};

// Btn &b — ссылка: функция работает с самой кнопкой, а не с её копией.
// Результат — b.state: «нажата» устанавливается, только если значение стабильно DEBOUNCE_MS.
void updateBtn(Btn &b) {
  bool raw = (digitalRead(b.pin) == LOW);        // LOW = нажата
  uint32_t now = millis();
  if (raw != b.raw) { b.raw = raw; b.tRaw = now; }          // значение дёрнулось
  if (b.state != b.raw && (now - b.tRaw) >= DEBOUNCE_MS) {  // стоит стабильно дольше DEBOUNCE_MS
    b.state = b.raw;
  }
}

uint8_t seq   = 0;   // счётчик пакетов (переполнение 255→0 — нормально)

void setup() {
  Serial.begin(115200);

  // INPUT_PULLUP — подтяжка к питанию внутри чипа. На ЙоТик она, судя по
  // документации, дублирует внешнюю подтяжку порта, но вреда не делает.
  const uint8_t pins[] = {PIN_UP, PIN_DOWN, PIN_LEFT, PIN_RIGHT};
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);   // range-for: перебор всех элементов массива
  pinMode(PIN_LED, OUTPUT);

  WiFi.mode(WIFI_STA);          // ДО чтения MAC и до esp_now_init()
  Serial.print("MAC пульта: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Ошибка esp_now_init");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};          // {} — обнулить всю структуру
  memcpy(peer.peer_addr, broadcastMac, 6);    // memcpy(куда, откуда, сколько байт)
  peer.channel = 0;                       // 0 = текущий канал радиомодуля
  peer.encrypt = false;                   // без шифрования (расширение: ключи PMK/LMK)
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Не удалось добавить пира");
    while (true) delay(1000);
  }
}

void loop() {
  // Опрос кнопок идёт на каждом проходе loop(), без delay().
  updateBtn(bUp); updateBtn(bDown); updateBtn(bLeft); updateBtn(bRight);

  static uint32_t tSend = 0;
  if (millis() - tSend < SEND_PERIOD_MS) return;   // ещё рано слать
  tSend = millis();

  RemotePacket p = {};
  p.magic = PKT_MAGIC;
  p.seq   = seq++;                    // сначала записали, потом увеличили
  p.robotId = TARGET_ROBOT;
  if (bUp.state)    p.buttons |= BTN_UP;      // |= ставит бит
  if (bDown.state)  p.buttons |= BTN_DOWN;
  if (bLeft.state)  p.buttons |= BTN_LEFT;
  if (bRight.state) p.buttons |= BTN_RIGHT;

  esp_err_t r = esp_now_send(broadcastMac, (const uint8_t *)&p, sizeof(p));
  // &p — адрес структуры; приведение к uint8_t* нужно, потому что функция шлёт «просто байты».
  // r == ESP_OK означает «принято в очередь отправки». У broadcast нет подтверждений
  // и повторов, поэтому о доставке узнать нельзя — на этом и держится защитная остановка робота.

  // LED3 горит, пока удерживается любое направление и отправка принята в очередь.
  digitalWrite(PIN_LED, (r == ESP_OK && p.buttons) ? HIGH : LOW);
}
