/* ═══════════════════════════════════════════════════════════════════
 *  GARDEN PUMP IOT CONTROLLER
 *  ────────────────────────────────────────────────────────────────
 *  Hardware:  NodeMCU ESP8266 + YF-B3 flow sensor + 30A relay module
 *  Purpose:   Auto-control ½ HP pump based on incoming water flow
 *             with local WiFi dashboard for monitoring
 *  
 *  Author:    Built for Rohit
 *  Version:   1.0
 *  ─────────────────────────────────────────────────────────────── */


// ──────────────── LIBRARIES ────────────────
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>


// ──────────────── PIN ASSIGNMENTS ────────────────
#define PIN_FLOW_SENSOR   D2   // GPIO4 — YF-B3 yellow wire (pulse input, interrupt)
#define PIN_RELAY         D1   // GPIO5 — Relay module IN pin
#define PIN_LED_BUILTIN   D4   // GPIO2 — onboard LED (status indicator)


// ──────────────── WIFI ACCESS POINT CONFIG ────────────────
const char* AP_SSID     = "GardenPump";
const char* AP_PASSWORD = "pump1234";       // change this if you want


// ──────────────── FLOW DETECTION TUNING ────────────────
// YF-B3: pulses per second = 11 × flow_rate (L/min)
const float   PULSES_PER_LITRE     = 660.0;   // 11 Hz × 60 sec = 660 pulses per litre

// ──────────────── MINIMUM FLOW (DRY-RUN SAFETY) ────────────────
// The ONLY absolute number in the system, and it is NOT pressure-tuned.
// It simply answers "is water physically present in the pipe?"
// Below this, there is effectively no water regardless of municipal
// pressure, so the pump must not run. A loose ~1 L/min works anywhere.
const float   MIN_FLOW             = 1.0;     // L/min — below this = "no water"
const uint16_t SUSTAIN_ON_SECS     = 3;       // seconds of water present before ON
const uint16_t SUSTAIN_OFF_SECS    = 10;      // seconds below MIN_FLOW before OFF
const uint16_t MIN_OFF_GAP_SECS    = 60;      // wait after OFF before next ON allowed
const uint32_t MAX_RUN_MINUTES     = 30;      // safety: maximum continuous run time

// ──────────────── STUCK / NO-BOOST PUMP DETECTION ────────────────
// Municipal pressure varies, so we do NOT use any absolute target.
// Instead we record the baseline flow (pump off) and require a
// RELATIVE increase once the pump runs. A stuck/air-locked pump
// produces ~0 % increase; a working pump produces a large increase,
// so even a loose multiplier separates the two with no site tuning.
const float    BOOST_MULTIPLIER    = 1.3;     // working pump must raise flow ≥ 30 %
const uint16_t STUCK_GRACE_SECS    = 12;      // wait this long after pump ON before judging
const uint16_t BOOST_CONFIRM_SECS  = 5;       // boost must hold for this many seconds
const uint16_t FAULT_COOLDOWN_SECS = 180;     // wait after a stuck-fault before retry (3 min)
const uint8_t  MAX_RETRIES         = 3;        // consecutive stuck retries before long lockout
const uint16_t LONG_LOCKOUT_SECS   = 1800;    // 30 min lockout after MAX_RETRIES failures


// ──────────────── EEPROM STORAGE LAYOUT ────────────────
const int EEPROM_SIZE              = 512;
const int ADDR_TOTAL_LITRES        = 0;       // uint32_t (4 bytes) — lifetime litres
const int ADDR_TOTAL_SECONDS       = 4;       // uint32_t (4 bytes) — lifetime run sec
const int ADDR_SESSION_COUNT       = 8;       // uint32_t (4 bytes) — total sessions
const int ADDR_SESSIONS_START      = 12;      // ring buffer for last 20 sessions
const int SESSION_ENTRY_SIZE       = 12;      // 4B start_min_ago, 4B duration_sec, 4B litres
const int MAX_SESSIONS_STORED      = 20;


// ──────────────── GLOBAL STATE ────────────────
volatile uint32_t pulseCount = 0;             // incremented in ISR

uint32_t lastPulseRead     = 0;               // millis() of last 1-sec sample
uint32_t pulsesLastSecond  = 0;               // pulses counted last second
float    currentFlowRate   = 0.0;             // current L/min

uint32_t sustainedOnTime   = 0;               // seconds of sustained flow
uint32_t sustainedOffTime  = 0;               // seconds of no flow
uint32_t lastOffEpoch      = 0;               // when pump last turned OFF
uint32_t sessionStartMs    = 0;               // when current session began
uint32_t sessionLitres     = 0;               // litres pumped this session (×1000 for int)
uint32_t bootMillis        = 0;               // boot time reference

bool     pumpOn            = false;
bool     manualOverride    = false;           // user clicked manual ON
bool     manualBlock       = false;           // user clicked manual OFF

// Stuck-pump fault tracking
bool     faultStuck        = false;           // TRUE if pump was detected stuck
uint32_t faultEpoch        = 0;               // millis() when fault occurred
uint32_t faultCount        = 0;               // how many times stuck-fault triggered
uint8_t  retryCount        = 0;               // consecutive failed retries
bool     longLockout       = false;           // TRUE during 30-min lockout

float    baselineFlow      = 0.0;             // flow rate just BEFORE pump started
float    boostConfirmTime  = 0;               // secs the boost has been confirmed good
bool     boostVerified     = false;           // TRUE once boost confirmed this session

uint32_t totalLitres       = 0;               // lifetime litres pumped
uint32_t totalSeconds      = 0;               // lifetime seconds run
uint32_t sessionCount      = 0;               // total session counter

// Ring buffer: stores last N sessions in RAM (also persisted to EEPROM)
struct SessionRecord {
  uint32_t startMinAgo;     // minutes ago when session started (at recording time)
  uint32_t durationSec;     // how long the session ran
  uint32_t litresPumped;    // litres pumped in session
};
SessionRecord sessions[MAX_SESSIONS_STORED];
int sessionWriteIndex      = 0;


// ──────────────── WEB SERVER ────────────────
ESP8266WebServer server(80);


// ════════════════════════════════════════════════════════════════════
//                          INTERRUPT HANDLER
// ════════════════════════════════════════════════════════════════════
void IRAM_ATTR onFlowPulse() {
  pulseCount++;
}


// ════════════════════════════════════════════════════════════════════
//                          EEPROM HELPERS
// ════════════════════════════════════════════════════════════════════
void saveStats() {
  EEPROM.put(ADDR_TOTAL_LITRES, totalLitres);
  EEPROM.put(ADDR_TOTAL_SECONDS, totalSeconds);
  EEPROM.put(ADDR_SESSION_COUNT, sessionCount);
  EEPROM.commit();
}

void loadStats() {
  EEPROM.get(ADDR_TOTAL_LITRES, totalLitres);
  EEPROM.get(ADDR_TOTAL_SECONDS, totalSeconds);
  EEPROM.get(ADDR_SESSION_COUNT, sessionCount);
  
  // Sanity check — first boot, values are 0xFFFFFFFF
  if (totalLitres  == 0xFFFFFFFF) totalLitres  = 0;
  if (totalSeconds == 0xFFFFFFFF) totalSeconds = 0;
  if (sessionCount == 0xFFFFFFFF) sessionCount = 0;
}

void saveSessionToEEPROM(uint32_t durationSec, uint32_t litres) {
  int addr = ADDR_SESSIONS_START + (sessionWriteIndex * SESSION_ENTRY_SIZE);
  uint32_t startMinAgo = 0;  // just recorded, 0 min ago
  EEPROM.put(addr,     startMinAgo);
  EEPROM.put(addr + 4, durationSec);
  EEPROM.put(addr + 8, litres);
  EEPROM.commit();
}

void loadSessionsFromEEPROM() {
  for (int i = 0; i < MAX_SESSIONS_STORED; i++) {
    int addr = ADDR_SESSIONS_START + (i * SESSION_ENTRY_SIZE);
    EEPROM.get(addr,     sessions[i].startMinAgo);
    EEPROM.get(addr + 4, sessions[i].durationSec);
    EEPROM.get(addr + 8, sessions[i].litresPumped);
    
    if (sessions[i].startMinAgo == 0xFFFFFFFF) sessions[i].startMinAgo = 0;
    if (sessions[i].durationSec == 0xFFFFFFFF) sessions[i].durationSec = 0;
    if (sessions[i].litresPumped == 0xFFFFFFFF) sessions[i].litresPumped = 0;
  }
}


// ════════════════════════════════════════════════════════════════════
//                          PUMP CONTROL
// ════════════════════════════════════════════════════════════════════
void turnPumpOn() {
  if (pumpOn) return;
  // Capture the baseline flow BEFORE the pump kicks in.
  // Water already flows from municipal pressure — this is our reference.
  baselineFlow = currentFlowRate;
  boostConfirmTime = 0;
  boostVerified = false;
  digitalWrite(PIN_RELAY, HIGH);
  digitalWrite(PIN_LED_BUILTIN, LOW);   // onboard LED ON (active low)
  pumpOn = true;
  sessionStartMs = millis();
  sessionLitres = 0;
  Serial.printf("[PUMP] ON — baseline flow was %.2f L/min\n", baselineFlow);
}

void turnPumpOff() {
  if (!pumpOn) return;
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_LED_BUILTIN, HIGH);  // onboard LED OFF
  pumpOn = false;
  
  // Record the session
  uint32_t durationSec = (millis() - sessionStartMs) / 1000;
  uint32_t litres = sessionLitres / 1000;
  
  if (durationSec >= 5) {  // ignore micro-sessions
    sessions[sessionWriteIndex] = { 0, durationSec, litres };
    saveSessionToEEPROM(durationSec, litres);
    sessionWriteIndex = (sessionWriteIndex + 1) % MAX_SESSIONS_STORED;
    
    totalSeconds += durationSec;
    totalLitres += litres;
    sessionCount++;
    saveStats();
    
    Serial.printf("[PUMP] OFF — session: %lu sec, %lu L\n", durationSec, litres);
  }
  
  lastOffEpoch = millis();
}


// ════════════════════════════════════════════════════════════════════
//                          WEB DASHBOARD HTML
// ════════════════════════════════════════════════════════════════════
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
.status-on{color:#3fb950}
.status-off{color:#8b949e}
.btn{display:inline-block;padding:10px 18px;border-radius:6px;font-weight:600;text-decoration:none;font-size:0.9rem;margin-right:8px;border:none;cursor:pointer;font-family:inherit}
.btn-on{background:#3fb950;color:#000}
.btn-off{background:#f85149;color:#fff}
.btn-reset{background:#30363d;color:#e6edf3}
.session{padding:8px;background:#0d1117;border-radius:4px;margin-bottom:6px;font-size:0.82rem;font-family:monospace}
.foot{text-align:center;color:#30363d;font-size:0.7rem;margin-top:20px}
.live-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#3fb950;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.3}}
</style></head><body>
<h1>🌱 Garden Pump</h1>
<div class="sub">Auto-control via flow sensor · v1.0</div>
)=====");

  // Status card
  html += "<div class=\"card\"><div class=\"label\">Current Status</div>";
  html += "<div class=\"big ";
  html += pumpOn ? "status-on\"><span class=\"live-dot\"></span> RUNNING" : "status-off\">⏸ IDLE";
  html += "</div>";
  
  // Stuck-pump fault alert
  if (faultStuck) {
    uint32_t totalWait = longLockout ? LONG_LOCKOUT_SECS : FAULT_COOLDOWN_SECS;
    uint32_t coolLeft = 0;
    uint32_t elapsed = (millis() - faultEpoch) / 1000;
    if (elapsed < totalWait) coolLeft = totalWait - elapsed;
    html += "<div style=\"margin-top:12px;background:#3d1a1a;border:1px solid #f85149;border-radius:6px;padding:10px;color:#ff7b72;font-size:0.85rem\">";
    if (longLockout) {
      html += "⛔ <b>LONG LOCKOUT</b><br>Pump failed to boost " + String(MAX_RETRIES) + " times in a row.<br>";
      html += "Pump likely jammed — please inspect.<br>";
    } else {
      html += "⚠ <b>PUMP NOT BOOSTING</b><br>Pump ran but flow did not increase enough.<br>";
      html += "Likely stuck / air-locked.<br>";
    }
    html += "Retry in " + String(coolLeft / 60) + "m " + String(coolLeft % 60) + "s · ";
    html += "Faults: " + String(faultCount) + " · Tries: " + String(retryCount);
    html += "</div>";
  }
  
  // Boost status (when pump running)
  if (pumpOn && !faultStuck) {
    html += "<div style=\"margin-top:12px;background:#1a2e1a;border:1px solid #3fb950;border-radius:6px;padding:8px;color:#56d364;font-size:0.8rem\">";
    if (boostVerified) {
      html += "✓ Boost verified — baseline " + String(baselineFlow, 1) + " → now " + String(currentFlowRate, 1) + " L/min";
    } else {
      html += "⏳ Verifying boost… baseline " + String(baselineFlow, 1) + " L/min, watching for increase";
    }
    html += "</div>";
  }
  
  // Flow rate
  html += "<div style=\"margin-top:14px\"><div class=\"label\">Live Flow</div>";
  html += "<div class=\"big\">" + String(currentFlowRate, 1) + " <span style=\"font-size:1rem;color:#8b949e\">L/min</span></div></div>";
  
  if (pumpOn) {
    uint32_t sessSec = (millis() - sessionStartMs) / 1000;
    html += "<div style=\"margin-top:10px\"><div class=\"label\">This Session</div>";
    html += "<div style=\"font-family:monospace\">" + String(sessSec / 60) + "m " + String(sessSec % 60) + "s · " + String(sessionLitres / 1000) + " L</div></div>";
  }
  html += "</div>";
  
  // Manual controls
  html += "<div class=\"card\"><div class=\"label\">Manual Override</div>";
  html += "<form method=\"POST\" action=\"/manual\" style=\"display:inline\"><button class=\"btn btn-on\" name=\"a\" value=\"on\">▶ Force ON</button></form>";
  html += "<form method=\"POST\" action=\"/manual\" style=\"display:inline\"><button class=\"btn btn-off\" name=\"a\" value=\"off\">■ Force OFF</button></form>";
  html += "<form method=\"POST\" action=\"/manual\" style=\"display:inline\"><button class=\"btn btn-reset\" name=\"a\" value=\"auto\">↻ Auto</button></form>";
  html += "<div style=\"margin-top:10px;font-size:0.75rem;color:#8b949e\">";
  if (manualOverride) html += "⚠ Manual ON active";
  else if (manualBlock) html += "⚠ Manual OFF blocking";
  else html += "✓ Automatic mode";
  html += "</div></div>";
  
  // Lifetime stats
  html += "<div class=\"card\"><div class=\"label\">Lifetime Stats</div>";
  html += "<div class=\"row\"><span>Total water pumped</span><span>" + String(totalLitres) + " L</span></div>";
  html += "<div class=\"row\"><span>Total run time</span><span>" + String(totalSeconds / 3600) + "h " + String((totalSeconds % 3600) / 60) + "m</span></div>";
  html += "<div class=\"row\"><span>Total sessions</span><span>" + String(sessionCount) + "</span></div>";
  html += "<div class=\"row\"><span>Stuck faults</span><span>" + String(faultCount) + "</span></div>";
  html += "<div class=\"row\"><span>System uptime</span><span>" + String((millis() - bootMillis) / 60000) + " min</span></div>";
  html += "</div>";
  
  // Recent sessions
  html += "<div class=\"card\"><div class=\"label\">Recent Sessions</div>";
  int count = 0;
  for (int i = 0; i < MAX_SESSIONS_STORED; i++) {
    int idx = (sessionWriteIndex - 1 - i + MAX_SESSIONS_STORED) % MAX_SESSIONS_STORED;
    if (sessions[idx].durationSec > 0) {
      uint32_t d = sessions[idx].durationSec;
      html += "<div class=\"session\">⏱ " + String(d / 60) + "m " + String(d % 60) + "s · 💧 " + String(sessions[idx].litresPumped) + " L</div>";
      count++;
      if (count >= 10) break;
    }
  }
  if (count == 0) html += "<div style=\"color:#8b949e;font-size:0.85rem\">No sessions yet</div>";
  html += "</div>";
  
  // Auto-refresh script
  html += "<div class=\"foot\">Auto-refreshes every 5 sec · NodeMCU ESP8266</div>";
  html += "<script>setTimeout(()=>location.reload(),5000)</script>";
  html += "</body></html>";
  
  return html;
}


// ════════════════════════════════════════════════════════════════════
//                          WEB HANDLERS
// ════════════════════════════════════════════════════════════════════
void handleRoot() {
  server.send(200, "text/html", dashboardHTML());
}

void handleManual() {
  String action = server.arg("a");
  
  if (action == "on") {
    manualOverride = true;
    manualBlock = false;
    turnPumpOn();
  } else if (action == "off") {
    manualOverride = false;
    manualBlock = true;
    turnPumpOff();
  } else if (action == "auto") {
    manualOverride = false;
    manualBlock = false;
  }
  
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNotFound() {
  server.sendHeader("Location", "/");
  server.send(303);
}


// ════════════════════════════════════════════════════════════════════
//                          SETUP
// ════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n[BOOT] Garden Pump IoT v1.0");
  
  bootMillis = millis();
  
  // Pin modes
  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  
  digitalWrite(PIN_RELAY, LOW);          // pump OFF at boot
  digitalWrite(PIN_LED_BUILTIN, HIGH);   // LED off (active low)
  
  // EEPROM init
  EEPROM.begin(EEPROM_SIZE);
  loadStats();
  loadSessionsFromEEPROM();
  Serial.printf("[BOOT] Loaded: %lu L lifetime, %lu sessions\n", totalLitres, sessionCount);
  
  // Attach flow sensor interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), onFlowPulse, RISING);
  Serial.println("[BOOT] Flow sensor interrupt attached on D2");
  
  // Start WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("[WIFI] AP started: ");
  Serial.println(AP_SSID);
  Serial.print("[WIFI] IP: ");
  Serial.println(apIP);
  Serial.println("[WIFI] Connect to WiFi 'GardenPump' and visit http://192.168.4.1");
  
  // Web server routes
  server.on("/", handleRoot);
  server.on("/manual", HTTP_POST, handleManual);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] Server started");
  
  lastPulseRead = millis();
}


// ════════════════════════════════════════════════════════════════════
//                          MAIN LOOP
// ════════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();
  
  uint32_t now = millis();
  
  // Sample flow every 1 second
  if (now - lastPulseRead >= 1000) {
    noInterrupts();
    pulsesLastSecond = pulseCount;
    pulseCount = 0;
    interrupts();
    
    // Convert pulses/sec → L/min (YF-B3: 11 Hz = 1 L/min)
    currentFlowRate = pulsesLastSecond / 11.0;
    
    // Track total litres if pump is on
    if (pumpOn) {
      // litres in this second = pulses / 660
      // ×1000 for milli-litre integer math
      sessionLitres += (pulsesLastSecond * 1000UL) / 660;
    }
    
    lastPulseRead = now;
    
    // Print every 5 sec
    static int printCounter = 0;
    if (++printCounter >= 5) {
      printCounter = 0;
      Serial.printf("[FLOW] %.2f L/min | pump:%s | manual:%s\n",
                    currentFlowRate,
                    pumpOn ? "ON" : "OFF",
                    manualOverride ? "ON" : (manualBlock ? "BLOCK" : "auto"));
    }
    
    // ──────────── CONTROL LOGIC ────────────
    
    if (manualOverride) {
      // User forced ON — keep it on regardless
      if (!pumpOn) turnPumpOn();
    }
    else if (manualBlock) {
      // User forced OFF — keep it off regardless
      if (pumpOn) turnPumpOff();
    }
    else {
      // ─── Automatic mode ───
      
      if (!pumpOn) {
        // Pump is OFF, check if we should turn ON
        
        // If a stuck-fault occurred recently, stay locked out during cooldown.
        // After MAX_RETRIES failures, use the much longer lockout instead.
        if (faultStuck) {
          uint32_t waitMs = longLockout
                            ? (LONG_LOCKOUT_SECS * 1000UL)
                            : (FAULT_COOLDOWN_SECS * 1000UL);
          if ((now - faultEpoch) >= waitMs) {
            Serial.println("[FAULT] Cooldown over — clearing fault, will retry");
            faultStuck = false;
            if (longLockout) {
              longLockout = false;
              retryCount = 0;   // reset after the long penalty
            }
          } else {
            // still cooling down — do not start pump
            sustainedOnTime = 0;
            yield();
            return;
          }
        }
        
        if (currentFlowRate >= MIN_FLOW) {
          sustainedOnTime++;
          
          // Check minimum gap since last OFF
          bool gapOK = (now - lastOffEpoch) >= (MIN_OFF_GAP_SECS * 1000UL);
          
          if (sustainedOnTime >= SUSTAIN_ON_SECS && gapOK) {
            Serial.println("[AUTO] Sustained flow detected — starting pump");
            turnPumpOn();
            sustainedOnTime = 0;
            sustainedOffTime = 0;
          }
        } else {
          sustainedOnTime = 0;
        }
      }
      else {
        // Pump is ON, check if we should turn OFF
        
        // Safety: maximum run time
        uint32_t runMinutes = (now - sessionStartMs) / 60000;
        if (runMinutes >= MAX_RUN_MINUTES) {
          Serial.println("[SAFETY] Max run time reached — forcing OFF");
          turnPumpOff();
          sustainedOnTime = 0;
          sustainedOffTime = 0;
          return;
        }
        
        // ─── STUCK / NO-BOOST DETECTION ───
        // Water already flows from municipal pressure (baselineFlow).
        // A working pump must BOOST flow significantly above baseline.
        // After the grace period, verify the boost actually happened.
        uint32_t runSecs = (now - sessionStartMs) / 1000;
        
        if (runSecs >= STUCK_GRACE_SECS && !boostVerified) {
          // Purely RELATIVE requirement — no absolute target, no site tuning.
          // Required flow = whatever the baseline was today × multiplier.
          float requiredFlow = baselineFlow * BOOST_MULTIPLIER;
          
          if (currentFlowRate >= requiredFlow) {
            // Boost is happening — confirm it holds for a few seconds
            boostConfirmTime++;
            if (boostConfirmTime >= BOOST_CONFIRM_SECS) {
              boostVerified = true;
              retryCount = 0;   // success — reset retry counter
              Serial.printf("[BOOST] OK — baseline %.1f → now %.1f L/min (req %.1f)\n",
                            baselineFlow, currentFlowRate, requiredFlow);
            }
          } else {
            // Flow did NOT rise enough — pump is stuck / not boosting
            boostConfirmTime = 0;
            Serial.printf("[FAULT] No boost! baseline %.1f, now %.1f, needed %.1f L/min — STUCK\n",
                          baselineFlow, currentFlowRate, requiredFlow);
            turnPumpOff();
            faultStuck = true;
            faultEpoch = now;
            faultCount++;
            retryCount++;
            
            // Too many consecutive failures → long lockout
            if (retryCount >= MAX_RETRIES) {
              longLockout = true;
              Serial.printf("[FAULT] %d consecutive failures — LONG LOCKOUT %d min\n",
                            retryCount, LONG_LOCKOUT_SECS / 60);
            }
            sustainedOnTime = 0;
            sustainedOffTime = 0;
            return;
          }
        }
        
        if (currentFlowRate < MIN_FLOW) {
          sustainedOffTime++;
          if (sustainedOffTime >= SUSTAIN_OFF_SECS) {
            Serial.println("[AUTO] Flow below minimum — water gone, pump OFF");
            turnPumpOff();
            sustainedOnTime = 0;
            sustainedOffTime = 0;
          }
        } else {
          sustainedOffTime = 0;
        }
      }
    }
  }
  
  yield();
}
