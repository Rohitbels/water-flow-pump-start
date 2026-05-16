/* ═══════════════════════════════════════════════════════════════════
 *  GARDEN PUMP IOT CONTROLLER  v2.0
 *  Hardware:  NodeMCU ESP8266 + YF-B3 flow sensor + 30A relay module
 *
 *  Logic:
 *    1. Water arrives with decent flow → start pump after a few seconds
 *    2. Pump must boost flow above baseline within grace period → verified
 *    3. If no boost → pump is stuck → wait 50 s → retry (indefinitely)
 *    4. When water stops → turn pump off → wait for next arrival
 * ══════════════════════════════════════════════════════════════════ */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>


// ──────────────── PINS ────────────────
#define PIN_FLOW_SENSOR   D2   // YF-B3 yellow wire (interrupt)
#define PIN_RELAY         D1   // Relay IN
#define PIN_LED_BUILTIN   D4   // Onboard LED (active LOW)


// ──────────────── WIFI ────────────────
const char* AP_SSID     = "GardenPump";
const char* AP_PASSWORD = "pump1234";


// ──────────────── FLOW SENSOR ────────────────
// YF-B3: 11 pulses/sec = 1 L/min  →  660 pulses = 1 litre
const float PULSES_PER_LITRE = 660.0;


// ──────────────── TUNING ────────────────
const float    START_FLOW       = 2.0;  // L/min — "decent flow" needed to start pump
const float    MIN_FLOW         = 1.0;  // L/min — below this the pump stops
const uint16_t SUSTAIN_ON_SECS  = 3;   // seconds of decent flow before starting
const uint16_t SUSTAIN_OFF_SECS = 10;  // seconds below MIN_FLOW before stopping
const float    BOOST_MULTIPLIER = 1.3; // pump must raise flow ≥ 30 % above baseline
const uint16_t STUCK_GRACE_SECS = 12;  // seconds after pump ON before checking boost
const uint16_t FAULT_WAIT_SECS  = 50;  // wait after a stuck-fault before retry
const uint32_t MAX_RUN_MINUTES  = 40;  // hard cap — stops pump regardless after this long


// ──────────────── EEPROM ────────────────
const int EEPROM_SIZE         = 256;
const int ADDR_TOTAL_LITRES   = 0;   // uint32_t — lifetime litres
const int ADDR_TOTAL_SECONDS  = 4;   // uint32_t — lifetime run seconds
const int ADDR_SESSION_COUNT  = 8;   // uint32_t — total sessions
const int ADDR_SESSIONS_START = 12;  // ring buffer — last 20 sessions
const int SESSION_ENTRY_SIZE  = 8;   // 4B duration_sec + 4B litres
const int MAX_SESSIONS_STORED = 20;


// ──────────────── STATE ────────────────
volatile uint32_t pulseCount    = 0;   // incremented in ISR

uint32_t lastPulseRead    = 0;
uint32_t pulsesLastSecond = 0;
float    currentFlowRate  = 0.0;

uint32_t sustainedOnTime  = 0;        // consecutive seconds above START_FLOW
uint32_t sustainedOffTime = 0;        // consecutive seconds below MIN_FLOW
uint32_t sessionStartMs   = 0;
uint32_t sessionLitres    = 0;        // milli-litres this session
uint32_t bootMillis       = 0;

bool     pumpOn         = false;
bool     manualOverride = false;
bool     manualBlock    = false;

bool     faultStuck = false;
uint32_t faultEpoch = 0;
uint32_t faultCount = 0;

float    baselineFlow  = 0.0;
bool     boostVerified = false;
uint8_t  boostSeenSecs = 0;    // consecutive seconds flow has been above required

uint32_t totalLitres  = 0;
uint32_t totalSeconds = 0;
uint32_t sessionCount = 0;

struct SessionRecord { uint32_t durationSec; uint32_t litresPumped; };
SessionRecord sessions[MAX_SESSIONS_STORED];
int sessionWriteIndex = 0;

ESP8266WebServer server(80);


// ──────────────── ISR ────────────────
void IRAM_ATTR onFlowPulse() { pulseCount++; }


// ──────────────── EEPROM HELPERS ────────────────
void saveStats() {
  EEPROM.put(ADDR_TOTAL_LITRES,  totalLitres);
  EEPROM.put(ADDR_TOTAL_SECONDS, totalSeconds);
  EEPROM.put(ADDR_SESSION_COUNT, sessionCount);
  EEPROM.commit();
}

void loadStats() {
  EEPROM.get(ADDR_TOTAL_LITRES,  totalLitres);
  EEPROM.get(ADDR_TOTAL_SECONDS, totalSeconds);
  EEPROM.get(ADDR_SESSION_COUNT, sessionCount);
  if (totalLitres  == 0xFFFFFFFF) totalLitres  = 0;
  if (totalSeconds == 0xFFFFFFFF) totalSeconds = 0;
  if (sessionCount == 0xFFFFFFFF) sessionCount = 0;
}

void saveSession(uint32_t durationSec, uint32_t litres) {
  int addr = ADDR_SESSIONS_START + sessionWriteIndex * SESSION_ENTRY_SIZE;
  EEPROM.put(addr,     durationSec);
  EEPROM.put(addr + 4, litres);
  EEPROM.commit();
}

void loadSessions() {
  for (int i = 0; i < MAX_SESSIONS_STORED; i++) {
    int addr = ADDR_SESSIONS_START + i * SESSION_ENTRY_SIZE;
    EEPROM.get(addr,     sessions[i].durationSec);
    EEPROM.get(addr + 4, sessions[i].litresPumped);
    if (sessions[i].durationSec  == 0xFFFFFFFF) sessions[i].durationSec  = 0;
    if (sessions[i].litresPumped == 0xFFFFFFFF) sessions[i].litresPumped = 0;
  }
}


// ──────────────── PUMP CONTROL ────────────────
void turnPumpOn() {
  if (pumpOn) return;
  baselineFlow   = currentFlowRate;
  boostVerified  = false;
  boostSeenSecs  = 0;
  sessionStartMs = millis();
  sessionLitres  = 0;
  digitalWrite(PIN_RELAY,       HIGH);
  digitalWrite(PIN_LED_BUILTIN, LOW);
  pumpOn = true;
  Serial.printf("[PUMP] ON — baseline %.2f L/min\n", baselineFlow);
}

void turnPumpOff() {
  if (!pumpOn) return;
  digitalWrite(PIN_RELAY,       LOW);
  digitalWrite(PIN_LED_BUILTIN, HIGH);
  pumpOn = false;

  uint32_t durationSec = (millis() - sessionStartMs) / 1000;
  uint32_t litres      = sessionLitres / 1000;

  if (durationSec >= 5) {
    sessions[sessionWriteIndex] = { durationSec, litres };
    saveSession(durationSec, litres);
    sessionWriteIndex = (sessionWriteIndex + 1) % MAX_SESSIONS_STORED;
    totalSeconds += durationSec;
    totalLitres  += litres;
    sessionCount++;
    saveStats();
    Serial.printf("[PUMP] OFF — %lu sec, %lu L\n", durationSec, litres);
  }
}


// ──────────────── WEB DASHBOARD ────────────────
String dashboardHTML() {
  String html = F(R"=====(<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Garden Pump</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,sans-serif;background:#0f1419;color:#e6edf3;padding:16px;max-width:600px;margin:0 auto}
h1{font-size:1.3rem;color:#f0a500;margin-bottom:4px}
.sub{color:#8b949e;font-size:0.78rem;margin-bottom:18px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-bottom:12px}
.label{font-size:0.7rem;color:#8b949e;letter-spacing:1.5px;text-transform:uppercase;margin-bottom:8px}
.big{font-size:2rem;font-weight:700;font-family:monospace}
.row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #21262d}
.row:last-child{border-bottom:none}
.row span:first-child{color:#8b949e;font-size:0.85rem}
.row span:last-child{font-weight:600;font-family:monospace}
.status-on{color:#3fb950}.status-off{color:#8b949e}
.btn{display:inline-block;padding:10px 18px;border-radius:6px;font-weight:600;font-size:0.9rem;margin-right:8px;border:none;cursor:pointer;font-family:inherit}
.btn-on{background:#3fb950;color:#000}.btn-off{background:#f85149;color:#fff}.btn-reset{background:#30363d;color:#e6edf3}
.session{padding:8px;background:#0d1117;border-radius:4px;margin-bottom:6px;font-size:0.82rem;font-family:monospace}
.foot{text-align:center;color:#30363d;font-size:0.7rem;margin-top:20px}
.live-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#3fb950;animation:blink 1s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0.3}}
</style></head><body>
<h1>🌱 Garden Pump</h1>
<div class="sub">Auto-control via flow sensor · v2.0</div>
)=====");

  // Status
  html += "<div class=\"card\"><div class=\"label\">Status</div><div class=\"big ";
  html += pumpOn ? "status-on\"><span class=\"live-dot\"></span> RUNNING" : "status-off\">⏸ IDLE";
  html += "</div>";

  if (faultStuck) {
    uint32_t elapsed  = (millis() - faultEpoch) / 1000;
    uint32_t coolLeft = (elapsed < FAULT_WAIT_SECS) ? (FAULT_WAIT_SECS - elapsed) : 0;
    html += "<div style=\"margin-top:12px;background:#3d1a1a;border:1px solid #f85149;border-radius:6px;padding:10px;color:#ff7b72;font-size:0.85rem\">";
    html += "⚠ <b>PUMP STUCK</b> — flow did not increase after start.<br>Retry in " + String(coolLeft) + "s · Faults: " + String(faultCount);
    html += "</div>";
  }

  if (pumpOn) {
    html += "<div style=\"margin-top:12px;background:#1a2e1a;border:1px solid #3fb950;border-radius:6px;padding:8px;color:#56d364;font-size:0.8rem\">";
    html += boostVerified
      ? "✓ Boost OK — baseline " + String(baselineFlow, 1) + " → now " + String(currentFlowRate, 1) + " L/min"
      : "⏳ Waiting to verify boost… baseline " + String(baselineFlow, 1) + " L/min";
    html += "</div>";
  }

  html += "<div style=\"margin-top:14px\"><div class=\"label\">Live Flow</div>";
  html += "<div class=\"big\">" + String(currentFlowRate, 1) + " <span style=\"font-size:1rem;color:#8b949e\">L/min</span></div></div>";

  if (pumpOn) {
    uint32_t s = (millis() - sessionStartMs) / 1000;
    html += "<div style=\"margin-top:10px\"><div class=\"label\">This Session</div>";
    html += "<div style=\"font-family:monospace\">" + String(s / 60) + "m " + String(s % 60) + "s · " + String(sessionLitres / 1000) + " L</div></div>";
  }
  html += "</div>";

  // Manual controls
  html += "<div class=\"card\"><div class=\"label\">Manual Override</div>";
  html += "<form method=\"POST\" action=\"/manual\" style=\"display:inline\"><button class=\"btn btn-on\"  name=\"a\" value=\"on\" >▶ Force ON</button></form>";
  html += "<form method=\"POST\" action=\"/manual\" style=\"display:inline\"><button class=\"btn btn-off\" name=\"a\" value=\"off\">■ Force OFF</button></form>";
  html += "<form method=\"POST\" action=\"/manual\" style=\"display:inline\"><button class=\"btn btn-reset\" name=\"a\" value=\"auto\">↻ Auto</button></form>";
  html += "<div style=\"margin-top:10px;font-size:0.75rem;color:#8b949e\">";
  if      (manualOverride) html += "⚠ Manual ON active";
  else if (manualBlock)    html += "⚠ Manual OFF active";
  else                     html += "✓ Automatic mode";
  html += "</div></div>";

  // Lifetime stats
  html += "<div class=\"card\"><div class=\"label\">Lifetime Stats</div>";
  html += "<div class=\"row\"><span>Total pumped</span><span>"    + String(totalLitres) + " L</span></div>";
  html += "<div class=\"row\"><span>Total run time</span><span>" + String(totalSeconds / 3600) + "h " + String((totalSeconds % 3600) / 60) + "m</span></div>";
  html += "<div class=\"row\"><span>Sessions</span><span>"       + String(sessionCount) + "</span></div>";
  html += "<div class=\"row\"><span>Stuck faults</span><span>"   + String(faultCount) + "</span></div>";
  html += "<div class=\"row\"><span>Uptime</span><span>"         + String((millis() - bootMillis) / 60000) + " min</span></div>";
  html += "</div>";

  // Recent sessions
  html += "<div class=\"card\"><div class=\"label\">Recent Sessions</div>";
  int shown = 0;
  for (int i = 0; i < MAX_SESSIONS_STORED && shown < 10; i++) {
    int idx = (sessionWriteIndex - 1 - i + MAX_SESSIONS_STORED) % MAX_SESSIONS_STORED;
    if (sessions[idx].durationSec > 0) {
      uint32_t d = sessions[idx].durationSec;
      html += "<div class=\"session\">⏱ " + String(d / 60) + "m " + String(d % 60) + "s · 💧 " + String(sessions[idx].litresPumped) + " L</div>";
      shown++;
    }
  }
  if (shown == 0) html += "<div style=\"color:#8b949e;font-size:0.85rem\">No sessions yet</div>";
  html += "</div>";

  html += "<div class=\"foot\">Auto-refreshes every 5 sec · NodeMCU ESP8266</div>";
  html += "<script>setTimeout(()=>location.reload(),5000)</script>";
  html += "</body></html>";
  return html;
}


// ──────────────── WEB HANDLERS ────────────────
void handleRoot()     { server.send(200, "text/html", dashboardHTML()); }
void handleNotFound() { server.sendHeader("Location", "/"); server.send(303); }

void handleManual() {
  String a = server.arg("a");
  if      (a == "on")   { manualOverride = true;  manualBlock = false; turnPumpOn();  }
  else if (a == "off")  { manualOverride = false; manualBlock = true;  turnPumpOff(); }
  else if (a == "auto") { manualOverride = false; manualBlock = false; }
  server.sendHeader("Location", "/");
  server.send(303);
}


// ──────────────── SETUP ────────────────
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n[BOOT] Garden Pump IoT v2.0");

  bootMillis = millis();

  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  pinMode(PIN_RELAY,        OUTPUT);
  pinMode(PIN_LED_BUILTIN,  OUTPUT);
  digitalWrite(PIN_RELAY,       LOW);
  digitalWrite(PIN_LED_BUILTIN, HIGH);

  EEPROM.begin(EEPROM_SIZE);
  loadStats();
  loadSessions();
  Serial.printf("[BOOT] Loaded: %lu L lifetime, %lu sessions\n", totalLitres, sessionCount);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), onFlowPulse, RISING);
  Serial.println("[BOOT] Flow sensor ready on D2");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 1);  // channel 1, hidden SSID
  Serial.printf("[WIFI] AP: %s (hidden)  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.println("[WIFI] Visit http://192.168.4.1");

  server.on("/",       handleRoot);
  server.on("/manual", HTTP_POST, handleManual);
  server.onNotFound(handleNotFound);
  server.begin();

  lastPulseRead = millis();
}


// ──────────────── MAIN LOOP ────────────────
void loop() {
  server.handleClient();

  uint32_t now = millis();

  // ── Sample flow once per second ──────────────────────────────
  if (now - lastPulseRead < 1000) { yield(); return; }

  noInterrupts();
  pulsesLastSecond = pulseCount;
  pulseCount = 0;
  interrupts();

  currentFlowRate = pulsesLastSecond / 11.0;            // pulses/sec ÷ 11 = L/min
  if (pumpOn) sessionLitres += (pulsesLastSecond * 1000UL) / 660;
  lastPulseRead = now;

  static int logTick = 0;
  if (++logTick >= 5) {
    logTick = 0;
    Serial.printf("[FLOW] %.2f L/min | pump:%s\n", currentFlowRate, pumpOn ? "ON" : "OFF");
  }

  // ── Manual overrides (bypass all automatic logic) ─────────────
  if (manualOverride) { if (!pumpOn) turnPumpOn();  yield(); return; }
  if (manualBlock)    { if (pumpOn)  turnPumpOff(); yield(); return; }

  // ── AUTOMATIC MODE ───────────────────────────────────────────

  if (!pumpOn) {

    // After a stuck fault, wait FAULT_WAIT_SECS before trying again
    if (faultStuck) {
      if ((now - faultEpoch) >= (FAULT_WAIT_SECS * 1000UL)) {
        Serial.println("[FAULT] Cooldown done — ready to retry");
        faultStuck = false;
      } else {
        sustainedOnTime = 0;
        yield();
        return;
      }
    }

    // Wait for decent, sustained flow before starting the pump
    if (currentFlowRate >= START_FLOW) {
      sustainedOnTime++;
      if (sustainedOnTime >= SUSTAIN_ON_SECS) {
        Serial.println("[AUTO] Decent flow confirmed — starting pump");
        turnPumpOn();
        sustainedOnTime  = 0;
        sustainedOffTime = 0;
      }
    } else {
      sustainedOnTime = 0;
    }

  } else {  // pump is ON

    // ── Hard cap ─────────────────────────────────────────────
    if ((now - sessionStartMs) >= (MAX_RUN_MINUTES * 60000UL)) {
      Serial.println("[SAFETY] Max run time reached — pump OFF");
      turnPumpOff();
      sustainedOnTime = sustainedOffTime = 0;
      yield(); return;
    }

    // ── Boost check: did the pump actually increase flow? ──────
    // Requires 2 consecutive seconds above threshold to verify —
    // filters a single pressure spike from faking a good boost.
    // A bad reading faults immediately — no grace on the fault side.
    uint32_t runSecs = (now - sessionStartMs) / 1000;

    if (runSecs >= STUCK_GRACE_SECS && !boostVerified) {
      if (currentFlowRate >= baselineFlow * BOOST_MULTIPLIER) {
        if (++boostSeenSecs >= 2) {
          boostVerified = true;
          Serial.printf("[BOOST] OK — %.1f → %.1f L/min\n", baselineFlow, currentFlowRate);
        }
      } else {
        boostSeenSecs    = 0;
        Serial.printf("[FAULT] No boost — baseline %.1f, now %.1f L/min — pump stuck\n",
                      baselineFlow, currentFlowRate);
        turnPumpOff();
        faultStuck       = true;
        faultEpoch       = now;
        faultCount++;
        sustainedOnTime  = 0;
        sustainedOffTime = 0;
        yield();
        return;
      }
    }

    // ── Stop when water is gone ────────────────────────────────
    if (currentFlowRate < MIN_FLOW) {
      sustainedOffTime++;
      if (sustainedOffTime >= SUSTAIN_OFF_SECS) {
        Serial.println("[AUTO] Water gone — pump OFF");
        turnPumpOff();
        sustainedOnTime  = 0;
        sustainedOffTime = 0;
      }
    } else {
      sustainedOffTime = 0;
    }
  }

  yield();
}
