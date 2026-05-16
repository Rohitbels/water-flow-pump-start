# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **Arduino/ESP8266 firmware project** — not a web or Node.js application. The primary artifact is `garden_pump_iot_v4.ino`, a single-file Arduino sketch for a NodeMCU ESP8266.

**Purpose:** Automatically control a ½ HP booster pump based on detected water flow from an intermittent municipal supply. The controller prevents dry-running and detects stuck/air-locked pumps.

## Build & Flash

There is no Makefile or CLI build system. Use the **Arduino IDE**:

1. Add ESP8266 board support via Boards Manager URL: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
2. Select board: **NodeMCU 1.0 (ESP-12E Module)**
3. Connect NodeMCU via micro-USB, select the COM port, click **Upload**
4. Monitor via Serial Monitor at **115200 baud**

Libraries used (all bundled with esp8266 Arduino core — no external installs needed):
- `ESP8266WiFi.h`
- `ESP8266WebServer.h`
- `EEPROM.h`

## Repository Structure

| File | Purpose |
|---|---|
| `garden_pump_iot_v4.ino` | The firmware — the only file that runs on hardware |
| `wiring.html` | Full circuit diagram + connection table (open in browser) |
| `README.md` | Hardware BOM, wiring summary, tuning guide |

The README mentions a `firmware/` subdirectory layout — the actual file is flat at the repo root.

## Firmware Architecture

The sketch is a **single-file state machine**. The full cycle in automatic mode:

1. Flow sensor pulses are counted in an ISR (`onFlowPulse`, must be `IRAM_ATTR`).
2. Every 1 second the main loop reads and resets `pulseCount` (interrupts briefly disabled), computes `currentFlowRate` in L/min, then runs control logic.
3. **Start**: if `currentFlowRate >= START_FLOW` for `SUSTAIN_ON_SECS` consecutive seconds, the pump turns on. `baselineFlow` is recorded at that moment.
4. **Boost check**: after `STUCK_GRACE_SECS` seconds, `currentFlowRate` must be `>= baselineFlow × BOOST_MULTIPLIER`. If it is → `boostVerified = true`, keep running. If not → fault: pump off, `faultStuck = true`, wait `FAULT_WAIT_SECS`, retry indefinitely.
5. **Stop**: if `currentFlowRate < MIN_FLOW` for `SUSTAIN_OFF_SECS` consecutive seconds, pump turns off. Back to step 3.

Manual override (`manualOverride` / `manualBlock`) bypasses all automatic logic — checked first, exits immediately.

**EEPROM persistence**: lifetime stats (litres, seconds, session count) and a 20-entry ring buffer of recent sessions survive power cuts. Layout is fixed by `ADDR_*` constants — do not reorder without clearing/migrating EEPROM.

**Web dashboard**: `dashboardHTML()` builds the full page as a String on every HTTP request. Auto-refreshes every 5 seconds. Served at `192.168.4.1` in AP mode.

## Key Tunable Constants

| Constant | Default | Purpose |
|---|---|---|
| `START_FLOW` | 2.0 L/min | Minimum "decent" flow to start the pump |
| `MIN_FLOW` | 1.0 L/min | Below this the pump stops (water gone) |
| `SUSTAIN_ON_SECS` | 3 s | Flow must hold before pump starts |
| `SUSTAIN_OFF_SECS` | 10 s | No-flow must hold before pump stops |
| `BOOST_MULTIPLIER` | 1.3 | Pump must raise flow ≥ 30 % above baseline |
| `STUCK_GRACE_SECS` | 12 s | Grace period for pump to spin up before boost check |
| `FAULT_WAIT_SECS` | 50 s | Wait after a stuck-fault before retrying |

Calibration: run the pump manually from the dashboard, note the baseline (pump off) and boosted (pump on) flow readings, then set `BOOST_MULTIPLIER` to about 60–70 % of the observed ratio.

## Hardware Pin Map

| NodeMCU Pin | GPIO | Connected to |
|---|---|---|
| D1 | GPIO5 | Relay IN |
| D2 | GPIO4 | YF-B3 yellow (pulse, interrupt) |
| D4 | GPIO2 | Onboard LED (active LOW) |

Relay jumper must be set to **HIGH** trigger.

## WiFi

The device runs as a **WiFi Access Point** (no router needed). SSID: `GardenPump`, password: `pump1234` (hardcoded in `AP_SSID`/`AP_PASSWORD` constants). Dashboard at `http://192.168.4.1`.

## Safety Rules — Do Not Break

### 1. Dry-run protection must never be bypassed in automatic mode

The pump motor can be damaged within seconds of running dry. The control logic enforces two independent guards:

- **`MIN_FLOW` gate**: the pump cannot start unless `currentFlowRate >= MIN_FLOW` has been sustained for `SUSTAIN_ON_SECS` consecutive seconds.
- **Boost verification**: after start, flow must rise to `baselineFlow × BOOST_MULTIPLIER` and hold for `BOOST_CONFIRM_SECS` seconds, or the pump is stopped as stuck/dry.

Any change to the start-up path, the threshold constants, or the boost check must preserve both guards. Do not add code paths that start the pump outside the sustain window, and do not lower `MIN_FLOW` below a value that reliably indicates water is physically present.

**Known gap:** `manualOverride` intentionally bypasses all automatic safety logic (the user is assumed to know what they are doing). Do not extend manual-override semantics into the automatic path.
