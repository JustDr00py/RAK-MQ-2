# RAK4631 Gas & Environmental Monitor

LoRaWAN sensor node built on RAK WisBlock: temperature/humidity (RAK1901)
+ combustible gas approximation (RAK12004, MQ-2), reporting over
ChirpStack v4 on US915 sub-band 2.

## Hardware

| Part | Role |
|---|---|
| RAK4631 | WisBlock Core (nRF52840 + SX1262, RUI3 firmware) |
| RAK19007 | WisBlock Base Board 2nd Gen |
| RAK1901 | Temperature/humidity sensor (Sensirion **SHTC3** chip) |
| RAK12004 | Gas sensor (Winsen **MQ-2**, read via onboard ADC121C021 over I2C) |
| 2x 18650 Li-ion (optional) | Battery backup / UPS - see Power notes below |

**RAK12004 must go in the base board's dedicated IO slot**, not one of
the smaller sensor slots A-D - it uses a 40-pin WisConnector, they use
24-pin.

### Power

This device is designed to run on **mains power** (5V via USB or the
base board's power input), with a 2-cell 18650 pack (in parallel, not
series - stay within the board's single-cell 2.8-4.2V input range) as
optional battery backup.

Why mains, not battery-primary: the MQ-2's heater draws ~150mA
continuously (~0.75W) regardless of activity. Two 18650s in parallel
(~5,000-7,000mAh combined) run this for roughly **~25 hours** under
continuous load - workable as an emergency UPS buffer, not viable as a
primary power source. See the WisBlock base board's docs for battery
requirements/limits before connecting a pack.

If you're troubleshooting a "R0 is infinite/NaN - open circuit" error
on first bring-up: confirm a battery or equivalent VBAT source is
present. The RAK12004's onboard 5V boost converter (which powers both
the MQ-2 heater and the ADC121C021 itself) is fed from VBAT, not the
board's regulated 3.3V logic rail - on USB-only power with no battery,
that rail may not come up.

## Arduino IDE setup

1. **Add the RAKwireless RUI3 board package** (Boards Manager, not
   Library Manager):
   - File → Preferences → Additional Boards Manager URLs, add RAKwireless's
     RUI3 package index URL (see RAKwireless's RUI3 Arduino setup docs for
     the current URL - it occasionally changes).
   - Tools → Board → Boards Manager → search "RAKwireless RUI3" → Install.
   - Tools → Board → RAKwireless RUI3 → select the RAK4631 board variant.
   - Tools → Port → select the board's serial port.

2. **Install required libraries** (Library Manager, `Ctrl+Shift+I` /
   `Cmd+Shift+I`):
   - **`SparkFun SHTC3 Humidity and Temperature Sensor Library`** - for
     RAK1901. (Not Adafruit_SHT31 - that's a different chip and won't
     talk to this sensor correctly.)
   - **RAK-MQx library** (search "RAK-MQx" or "ADC121C021") - for
     RAK12004. Provides the `ADC121C021` class used in the sketch.
   - `Wire.h` is built into the Arduino core - no install needed.

3. **Fedora-specific note:** if uploading fails with
   `ModuleNotFoundError: No module named 'serial'`, the RAK uploader's
   Python dependency is missing:
   ```
   sudo dnf install python3-pyserial
   ```
   or, if that package isn't available:
   ```
   pip3 install --user pyserial
   ```

## LoRaWAN provisioning

Set credentials via AT commands over serial (Serial Monitor, 115200 baud)
before first join, unless already provisioned:

```
AT+DEUI=<16 hex chars>
AT+APPEUI=<16 hex chars>
AT+APPKEY=<32 hex chars>
```

- `AT+DEUI=?` / `AT+APPEUI=?` / `AT+APPKEY=?` read current values.
- DevEUI is usually factory-set on the RAK4631 - check before overwriting.
- Confirm join status any time with `AT+NJS=?` (`1` = joined).

This sketch is configured for **US915, sub-band 2** (channels 8-15 +
500kHz channel 65) and **LoRaWAN Class C** (continuous RX, so downlink
config changes land immediately - appropriate since this is
mains-powered, not battery-primary).

## ChirpStack v4 setup

1. Device Profile → Codec tab → set Codec type to **JavaScript functions**.
2. Paste the full contents of `chirpstack_v4_decoder.js` (includes both
   `decodeUplink` and `encodeDownlink`).
3. Uplinks arrive on **FPort 2**. Downlink config goes out on **FPort 10**
   - see `downlinks.md` for the full downlink command reference
     (short hex commands, full config payload, and worked examples).

## Calibration

- **First use / first flash:** the sketch runs a full calibration on
  boot - a 5-minute warm-up followed by repeated sampling until the
  sensor's readings stabilize (or it times out and disables gas
  reporting for that boot, which usually means a wiring/power/hardware
  issue worth investigating rather than a timing fluke).
- **Ordinary reboots** load the last validated calibration from flash
  instead of recalibrating blind - this avoids the device accidentally
  calibrating its "clean air" baseline against contaminated air if it
  happens to reboot during real gas/cooking activity.
- **To force a fresh calibration** (e.g., after physical hardware
  changes, or periodic recalibration), send downlink `01` on FPort 10.
  **The air must genuinely be clean when you do this** - see
  `downlinks.md` for details and the full warning.

## Known limitations

- The MQ-2's rated detection range is **300-10,000 ppm** per Winsen's
  datasheet - readings below 300 ppm are outside the sensor's
  characterized range and shouldn't be treated as precise.
- The regression curve used is Winsen's **LPG/butane** curve (RAK's own
  example constants). There is no official Winsen methane curve for the
  MQ-2 - butane/LPG readings are reasonably curve-matched, methane
  readings are a rougher cross-gas approximation.
- **This is not a certified life-safety gas detector.** For any
  application where a gas alarm needs to satisfy code requirements,
  insurance requirements, or actual occupant safety (e.g. commercial
  kitchens, multi-tenant residential), use a UL 2075 / UL 1484 listed
  detector (e.g. Macurco GD-series) as the authoritative alarm, wired
  into a WisBlock digital input module (e.g. RAK13001) if you want that
  alarm relayed over LoRaWAN. This sketch's readings are supplementary
  trend/monitoring data, not a safety system.
- Temperature/humidity correction on the gas reading is **disabled** -
  the RAK1901, mounted near the RAK12004, reads meaningfully hotter than
  true ambient due to heat from the MQ-2's own heater, which made the
  correction actively wrong. Temp/humidity are still reported for their
  own value as environmental data.

## Files

| File | Purpose |
|---|---|
| `rak4631_rak1901_rak12004.ino` | Main sketch |
| `chirpstack_v4_decoder.js` | ChirpStack v4 codec (`decodeUplink` + `encodeDownlink`) |
| `downlinks.md` | Downlink command reference and examples |
| `i2c_scanner.ino` | Standalone diagnostic - lists all I2C addresses on the bus |
