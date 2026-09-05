# Downlink Reference

Downlinks for this device are sent on **FPort 10**. Nothing else is
listened to for configuration - sensor data goes out on FPort 2, but no
downlink logic is attached to that port.

The device is configured as **LoRaWAN Class C**, so downlinks are
delivered essentially immediately rather than waiting for the device's
next scheduled uplink.

There are two payload formats, distinguished purely by length:

| Length | Format |
|---|---|
| 1 byte | Short command (a fixed action, no parameters) |
| 7 bytes | Full config (set specific values) |

Anything else (0 bytes, 2-6 bytes, 8+ bytes, or an unrecognized single
byte) is logged by the device and otherwise ignored.

---

## Short commands (1 byte)

Send these as a raw hex payload on FPort 10 - in ChirpStack v4, use the
device's **Downlink queue**, choose the raw/hex payload entry (not the
JSON codec form), set FPort to `10`, and paste the hex byte below.

| Hex | Command | Effect |
|---|---|---|
| `01` | Recalibrate | Triggers a fresh R0 calibration immediately (blocking on the device - the 5 minute warm-up + stability sampling runs before it returns). Applies for the current session only - it does **not** persist across a reboot; the device recalibrates fresh on every boot regardless. (An earlier version persisted R0 to flash; it was removed after it corrupted RUI3's BLE config - see README.) |
| `02` | Reset to defaults | Resets poll interval, uplink heartbeat interval, and alert threshold back to their compiled-in defaults (60s / 600s / 2000ppm). Does **not** touch calibration/R0. |

### ⚠️ Before sending `01`

The device has no way to know whether the air is actually clean when it
receives this command. **Confirm the space is clear of cooking activity,
cleaning products, aerosols, etc. before sending it** - a recalibration
performed in contaminated air will be saved as the new "clean air"
baseline and used for every reading afterward, silently skewing all
subsequent ppm values until the next recalibration.

### Examples

**Raw hex (any LNS raw-downlink field):**
```
01
```
```
02
```

---

## Full config (7 bytes)

Sets specific values instead of triggering a fixed action. Byte layout,
big-endian:

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0-1 | Poll interval | uint16, seconds | How often the device reads sensors locally. Clamped device-side to 5s-3600s. |
| 2-3 | Uplink heartbeat interval | uint16, seconds | How often it transmits when no alert is active. Clamped device-side to 30s-86400s (24h). |
| 4-5 | Alert threshold | uint16, ppm × 10 | e.g. `5000` = 500.0 ppm, `20000` = 2000.0 ppm. Compared directly against `methane_ppm_approx`. |
| 6 | Recalibrate flag | uint8 | `1` = trigger a fresh calibration as part of this downlink (same clean-air caveat as command `01` above). `0` = leave calibration untouched. |

Any submitted poll/uplink value outside the clamped range is silently
clamped to the nearest bound, not rejected - check the device's Serial
log (`Config updated: ...`) to confirm what was actually applied.

### Using the JSON codec (`encodeDownlink`)

If your codec script (`chirpstack_v4_decoder.js`) is loaded into the
device profile's Codec tab, ChirpStack's downlink queue lets you submit
a JSON object instead of hand-building the 7 raw bytes:

```json
{
  "pollIntervalSec": 60,
  "uplinkIntervalSec": 600,
  "alertThresholdPpm": 2000,
  "recalibrate": false
}
```

### Examples

**Set poll to 30s, heartbeat to 5 min, alert at 1500 ppm, no recalibration:**
```json
{
  "pollIntervalSec": 30,
  "uplinkIntervalSec": 300,
  "alertThresholdPpm": 1500,
  "recalibrate": false
}
```
Equivalent raw hex: `001E012C3A9800`
(30 = `0x001E`, 300 = `0x012C`, 1500×10=15000 = `0x3A98`, recalibrate=`0x00`)

**Set alert to 2500 ppm and recalibrate now, leaving poll/uplink at their current device-side values:**
```json
{
  "pollIntervalSec": 60,
  "uplinkIntervalSec": 600,
  "alertThresholdPpm": 2500,
  "recalibrate": true
}
```
Equivalent raw hex: `003C025861A801`
(60 = `0x003C`, 600 = `0x0258`, 2500×10=25000 = `0x61A8`, recalibrate=`0x01`)

> Note: the device does not merge partial updates - the full config
> downlink always sets all three values (poll, uplink, alert threshold)
> to whatever is in that payload. If you only want to change one
> setting, include the other two at their current known values rather
> than assuming they'll be left alone.

---

## Confirming a downlink was applied

Watch the device's Serial output. Every accepted downlink logs one of:

```
Command 0x01: recalibrate requested via downlink.
Command 0x02: poll/uplink/alertThreshold reset to compiled-in defaults.
Config updated: poll=30000ms uplink=300000ms alertThreshold=1500.0ppm recalibrate=0
```

If nothing was logged after sending a downlink, check:
- FPort is set to `10` (not 2, and not left at the LNS default of 1)
- Payload length is exactly 1 or 7 bytes
- The device actually received it - Class C should deliver quickly, but confirm the device shows as joined/online in ChirpStack
