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

The sketch is a **single-file state machine** with these main sections:

- **ISR** (`onFlowPulse`): hardware interrupt increments `pulseCount` on every sensor pulse — must be in IRAM (`IRAM_ATTR`).
- **1-second loop tick**: reads and resets `pulseCount` (with interrupts disabled), computes `currentFlowRate` in L/min, then runs control logic.
- **Control logic** (inside the tick): a three-way branch — `manualOverride`, `manualBlock`, or automatic mode. Automatic mode checks flow thresholds to start/stop the pump, and runs the stuck-pump detection state machine.
- **Boost verification**: after `STUCK_GRACE_SECS`, checks that `currentFlowRate >= baselineFlow × BOOST_MULTIPLIER`. If not met for `BOOST_CONFIRM_SECS`, declares fault → `turnPumpOff()` → cooldown → retry. After `MAX_RETRIES` failures → long lockout.
- **EEPROM persistence**: lifetime stats (litres, seconds, session count) and a 20-entry ring buffer of recent sessions survive power cuts. EEPROM layout is fixed by the `ADDR_*` constants — do not reorder without migrating existing data.
- **Web dashboard**: `dashboardHTML()` builds the full page as a String on every request. Auto-refreshes every 5 seconds via JS. Served on port 80 in AP mode at `192.168.4.1`.

## Key Tunable Constants

All tuning constants are at the top of the `.ino` file. The most important ones to calibrate per-site:

| Constant | Default | When to change |
|---|---|---|
| `BOOST_MULTIPLIER` | 1.3 | Measure actual baseline vs boosted flow on the dashboard, set to ~60-70% of observed ratio |
| `MIN_FLOW` | 1.0 L/min | "Is water present?" threshold — not pressure-tuned, safe as-is |
| `FAULT_COOLDOWN_SECS` | 180 | Retry wait after stuck-pump fault |
| `MAX_RETRIES` | 3 | Consecutive failures before long lockout |

## Hardware Pin Map

| NodeMCU Pin | GPIO | Connected to |
|---|---|---|
| D1 | GPIO5 | Relay IN |
| D2 | GPIO4 | YF-B3 yellow (pulse, interrupt) |
| D4 | GPIO2 | Onboard LED (active LOW) |

Relay jumper must be set to **HIGH** trigger.

## WiFi

The device runs as a **WiFi Access Point** (no router needed). SSID: `GardenPump`, password: `pump1234` (hardcoded in `AP_SSID`/`AP_PASSWORD` constants). Dashboard at `http://192.168.4.1`.
