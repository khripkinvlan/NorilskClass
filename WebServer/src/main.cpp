#include <WiFi.h>
#include <WebServer.h>

const char* SSID = "ESP32-Clicker-01";
const int LED = 18;
const int BTN = 0;

WebServer server(80);
int counter = 0;

const char PAGE[] = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Clicker</title>
<style>body{font-family:sans-serif;text-align:center;margin-top:10vh}
#n{font-size:20vw}button{font-size:8vw;padding:2vw 8vw;margin:1vw}</style></head>
<body><div id="n">0</div>
<button onclick="send('/click')">Click!</button>
<button onclick="send('/reset')">Reset</button>
<script>
async function send(url){
  const r = await fetch(url, {cache: 'no-store'});
  document.getElementById('n').textContent = await r.text();
}
setInterval(() => send('/count'), 1000);
</script></body></html>
)rawliteral";

void sendNoCache(int code, const char* type, const String& body) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(code, type, body);
}

void handleRoot()  { Serial.println("[HTTP] GET /"); sendNoCache(200, "text/html", PAGE); }

void handleClick() {
  counter++;
  int ledLevel = counter % 2;
  Serial.printf("[HTTP] GET /click -> counter=%d, LED write=%d\n", counter, ledLevel);
  digitalWrite(LED, ledLevel);
  Serial.printf("[LED ] read back = %d\n", digitalRead(LED));
  sendNoCache(200, "text/plain", String(counter));
}

void handleCount() {
  Serial.printf("[HTTP] GET /count -> %d\n", counter);
  sendNoCache(200, "text/plain", String(counter));
}

void handleReset() {
  counter = 0;
  digitalWrite(LED, LOW);
  Serial.println("[HTTP] GET /reset -> counter=0");
  sendNoCache(200, "text/plain", String(counter));
}

void handleNotFound() {
  Serial.printf("[HTTP] 404: %s %s\n",
    server.method() == HTTP_GET ? "GET" : "???",
    server.uri().c_str());
  server.send(404, "text/plain", "Not Found");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("================ BOOT ================");
  Serial.printf("Chip:   %s rev %d, cores %d\n",
    ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());
  Serial.printf("Flash:  %u bytes\n", (unsigned)ESP.getFlashChipSize());
  Serial.printf("Heap:   %u\n", (unsigned)ESP.getFreeHeap());

  // === LED self-test ===
  pinMode(LED, OUTPUT);
  Serial.printf("[LED ] Self-test on GPIO%d (5 blinks)...\n", LED);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, HIGH); delay(200);
    digitalWrite(LED, LOW);  delay(200);
  }
  Serial.println("[LED ] Test done. Нет вспышек? -> не тот пин или active-LOW.");
  Serial.println("[LED ] Если не мигает — попробуйте LED = 2 / 4 / 5 / 15 и active-LOW.");

  // === Button ===
  pinMode(BTN, INPUT_PULLUP);
  Serial.printf("[BTN ] GPIO%d initial: %s\n",
    BTN, digitalRead(BTN) ? "HIGH (released)" : "LOW (pressed?)");

  // === WiFi AP ===
  WiFi.softAP(SSID);
  Serial.printf("[WiFi] AP '%s' started, IP = %s\n",
    SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("[WiFi] MAC = %s\n", WiFi.macAddress().c_str());

  // === Routes ===
  server.on("/",       handleRoot);
  server.on("/click",  handleClick);
  server.on("/count",  handleCount);
  server.on("/reset",  handleReset);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] Server on port 80");
  Serial.println("=====================================");
  Serial.println("Команды в Serial: 'l'=toggle LED, 'r'=reset, 's'=status");
  Serial.println("=====================================");
}

void loop() {
  server.handleClient();

  // --- Serial commands ---
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'l' || c == 'L') {
      static int l = 0; l = !l;
      digitalWrite(LED, l);
      Serial.printf("[CMD ] LED <- %d, read=%d\n", l, digitalRead(LED));
    } else if (c == 'r' || c == 'R') {
      counter = 0; digitalWrite(LED, LOW);
      Serial.println("[CMD ] counter reset");
    } else if (c == 's' || c == 'S') {
      Serial.printf("[STAT] counter=%d LED_pin=%d LED_read=%d BTN_read=%d heap=%u\n",
        counter, LED, digitalRead(LED), digitalRead(BTN), (unsigned)ESP.getFreeHeap());
    }
  }

  // --- Raw button state trace (every edge) ---
  static bool lastRawBtn = HIGH;
  bool rawBtn = digitalRead(BTN);
  if (rawBtn != lastRawBtn) {
    Serial.printf("[BTN ] raw: %s -> %s (t=%lu ms)\n",
      lastRawBtn ? "HIGH" : "LOW", rawBtn ? "HIGH" : "LOW",
      (unsigned long)millis());
    lastRawBtn = rawBtn;
  }

  // --- Debounce + edge detection ---
  static bool stableBtn = HIGH;
  static unsigned long lastChange = 0;
  static bool counted = false;

  if (rawBtn != stableBtn) {
    lastChange = millis();
    stableBtn = rawBtn;
    counted = false;
  } else if (!counted && stableBtn == LOW && (millis() - lastChange) > 50) {
    counter++;
    counted = true;
    int ledLevel = counter % 2;
    digitalWrite(LED, ledLevel);
    Serial.printf("[BTN ] PRESSED -> counter=%d, LED write=%d, read=%d\n",
      counter, ledLevel, digitalRead(LED));
  }
}