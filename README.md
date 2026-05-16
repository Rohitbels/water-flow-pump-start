# 🌱 Garden Pump IoT Controller

Automated water boosting system for a garden line fed by an intermittent municipal supply. A NodeMCU (ESP8266) watches a flow sensor on the inlet pipe, runs a ½ HP booster pump only when water is actually flowing, verifies the pump is genuinely boosting (not stuck), and serves a local web dashboard for monitoring — all with **no internet required**.

---

## The Problem

- Municipal water arrives at **unpredictable times** (no fixed schedule).
- Water reaches the garden on its own due to municipal pressure, but the flow is **weak**.
- A ½ HP booster pump improves the flow, but it should run **only when water is present** — running it dry damages the motor.
- Off-the-shelf automatic pump controllers (CRI CPH-15 etc.) assume a **closed, on-demand plumbing system** (taps closed by default). They do **not** work for an always-open garden line, because they rely on pressure build-up that never happens when the outlet is open.
- The pump also **occasionally gets stuck** (air-lock / impeller jam) — it spins but doesn't move water. This must be detected and handled.

## The Solution

A flow-sensor-driven controller:

1. A **YF-B3 flow sensor** on the inlet pipe detects water actually moving.
2. A **NodeMCU ESP8266** reads the sensor pulses and decides when to run the pump.
3. A **30 A relay module** switches the pump's 230 V supply.
4. The controller **verifies the pump is boosting** — it records the baseline flow (municipal pressure alone) before starting, then confirms flow increases significantly once the pump runs. If it doesn't, the pump is stuck → emergency stop + automatic retry.
5. A **local WiFi access point + web dashboard** shows live status and run history. No internet, no cloud, no app needed.

---

## How It Works

```
Municipal line → Strainer → YF-B3 sensor → Pump → (old CPH-15 as check valve) → Garden
                                  │
                                  ▼
                            NodeMCU ESP8266
                          (reads flow pulses)
                                  │
                                  ▼
                           30 A Relay module
                                  │
                                  ▼
                         Pump 230 V ON / OFF
```

### Control logic

1. Sensor detects sustained flow above threshold → decide to start pump.
2. **Record baseline flow** (water already moving from municipal pressure).
3. Turn pump ON.
4. Wait a short grace period for the pump to spin up.
5. **Verify boost:** flow must rise to at least `BOOST_MULTIPLIER × baseline` **and** `baseline + BOOST_MIN_ABSOLUTE` L/min, held for a few seconds.
   - **Boost confirmed** → pump is working, keep running.
   - **No boost** → pump is stuck → emergency stop, cool down, retry.
6. After several consecutive failed retries → long lockout + "inspect pump" alert on the dashboard.
7. When flow stops (water supply ended) → pump turns OFF.

---

## Hardware

| Component | Spec | Notes |
|---|---|---|
| NodeMCU ESP8266 | v1.0 / ESP-12E | The controller / brain + WiFi |
| YF-B3 flow sensor | G½" brass, 1–25 L/min | Hall-effect pulse output, ~11 Hz per L/min |
| Relay module | 30 A, 5 V coil, optocoupler | High/Low trigger jumper set to **HIGH** |
| Pump | ½ HP, 230 V single phase | The booster being controlled |
| 5 V USB adapter | ≥1 A (3 A used here) | Powers the NodeMCU via micro-USB |
| Plumbing | ½" BSP brass unions ×2, ½" Y-strainer, PTFE tape | Strainer protects the sensor turbine |
| Enclosure | IP65 box, cable glands | Weatherproofing for outdoor install |

> The previously-purchased **CRI CPH-15** is retained inline only as a mechanical check valve (prevents backflow). Its own electronics are not relied upon.

### Wiring summary

**NodeMCU → Relay**

| NodeMCU | Relay |
|---|---|
| VIN (5 V) | VCC |
| GND | GND |
| D1 | IN |

**NodeMCU → YF-B3 sensor**

| NodeMCU | Sensor wire |
|---|---|
| VIN (5 V) | RED |
| GND | BLACK |
| D2 | YELLOW (pulse) |

**Relay → Pump (230 V AC)**

| From | To |
|---|---|
| Mains Live | Relay COM |
| Relay NO | Pump Live |
| Mains Neutral | Pump Neutral (direct) |
| Mains Earth | Pump body (mandatory) |

See the wiring diagrams in this repo for the full picture.

---

## Firmware

Arduino sketch for the NodeMCU (ESP8266 core). Key features:

- WiFi **Access Point** mode — phone connects directly, no router/internet.
- Local **web dashboard** at `http://192.168.4.1`.
- Hardware-interrupt pulse counting for accurate flow measurement.
- **Boost verification** to detect a stuck / air-locked pump.
- Automatic **retry with cooldown**, escalating to a long lockout after repeated failures.
- **Manual override** (Force ON / Force OFF / Auto) from the dashboard.
- Run history + lifetime stats persisted to **EEPROM** (survives power cuts).
- Safety caps: minimum off-gap, maximum continuous run time.

### Key tunable constants (top of the sketch)

| Constant | Default | Meaning |
|---|---|---|
| `FLOW_THRESHOLD_ON` | 2.0 L/min | Sustained flow needed to start the pump |
| `FLOW_THRESHOLD_OFF` | 0.5 L/min | Flow below this stops the pump |
| `SUSTAIN_ON_SECS` | 3 | Seconds of flow before starting |
| `SUSTAIN_OFF_SECS` | 10 | Seconds without flow before stopping |
| `BOOST_MULTIPLIER` | 1.5 | Boosted flow must be ≥ this × baseline |
| `BOOST_MIN_ABSOLUTE` | 2.0 L/min | …and ≥ baseline + this |
| `STUCK_GRACE_SECS` | 12 | Pump spin-up grace before judging boost |
| `BOOST_CONFIRM_SECS` | 5 | Boost must hold this long to confirm |
| `FAULT_COOLDOWN_SECS` | 180 | Wait after a stuck-fault before retry |
| `MAX_RETRIES` | 3 | Failures before the long lockout |
| `LONG_LOCKOUT_SECS` | 1800 | Long lockout duration (needs inspection) |
| `MAX_RUN_MINUTES` | 30 | Hard cap on continuous run time |

> **Calibration required:** measure your real baseline (pump off) and boosted (pump on) flow on the dashboard, then set `BOOST_MULTIPLIER` to roughly 60–70 % of the observed ratio so normal variation doesn't false-trigger but a stuck pump (ratio ≈ 1) is reliably caught.

---

## Accessing the Dashboard

1. Power the NodeMCU (USB adapter).
2. On a phone, join WiFi network **`GardenPump`** (password in the sketch, default `pump1234`).
3. Ignore the "no internet" warning — it's a local-only network.
4. Open a browser to **`http://192.168.4.1`**.
5. Optionally "Add to Home screen" for one-tap access.

The dashboard shows: live status, current flow, boost-verification state, manual controls, lifetime totals, fault counts, and recent run sessions. It auto-refreshes every 5 seconds.

---

## Repository Layout

```
.
├── README.md                      # this file
├── firmware/
│   └── garden_pump_iot_v3.ino     # current NodeMCU sketch (boost verification)
├── diagrams/
│   ├── wiring-diagram.html        # full circuit + connection table
│   ├── nodemcu-relay-wiring.html  # NodeMCU ↔ relay only
│   └── nodemcu-sensor-wiring.html # NodeMCU ↔ YF-B3 only
└── docs/                          # (optional) notes, photos, datasheets
```

> Adjust paths to match how you organise the files when you commit them.

---

## Build & Flash

1. Install the **Arduino IDE**.
2. Add ESP8266 board support: Preferences → Additional Boards Manager URLs →
   `http://arduino.esp8266.com/stable/package_esp8266com_index.json`,
   then Boards Manager → install **esp8266**.
3. Select board: **NodeMCU 1.0 (ESP-12E Module)**.
4. Open `firmware/garden_pump_iot_v3.ino`.
5. Connect the NodeMCU via micro-USB, pick the COM port, click **Upload**.
6. Open Serial Monitor at **115200 baud** to watch logs during testing.

---

## Testing Checklist

- [ ] Bench test (no mains, no pump): dashboard loads, manual ON/OFF toggles relay (audible click + LED).
- [ ] Relay trigger jumper set to **HIGH** (Force ON → relay active, Force OFF → relay off).
- [ ] Sensor test: water through the YF-B3 shows a sensible L/min on the dashboard.
- [ ] Measure baseline (pump off) and boosted (pump on) flow; set `BOOST_MULTIPLIER`.
- [ ] Dry/stuck test (briefly, carefully): pump on with no real boost → controller emergency-stops after the grace period and shows the fault.
- [ ] Leak-check all plumbing joints before powering the pump.
- [ ] Earth wire connected to the pump body.

---

## Safety

⚠️ This project switches **230 V AC** controlling a motor.

- Always switch off the mains MCB before touching any wiring; verify with a tester.
- The pump body **must be earthed**.
- Install a suitable **MCB** upstream of the system.
- Keep low-voltage (NodeMCU/sensor) wiring physically separated from 230 V wiring inside the enclosure.
- Use an **IP65 enclosure** with cable glands facing downward for an outdoor install.
- If unsure about the mains wiring, have a licensed electrician do the final connections.

---

## Status & Roadmap

**Working**
- Flow-based auto start/stop
- Boost verification / stuck-pump detection
- Retry + escalating lockout
- Local dashboard, EEPROM-persisted stats

**Possible future additions**
- Timestamped fault/session log
- Optional DS3231 RTC for real wall-clock timestamps
- Optional Telegram/notification support if internet is ever available at the site
- Watered-volume per day/week graphs on the dashboard

---

## License

Choose and add a license (e.g. MIT) when you publish the repo.
