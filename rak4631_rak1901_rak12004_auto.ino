/**
 * RAK4631 (RUI3) + RAK1901 (SHTC3 temp/humidity) + RAK12004 (MQ-2 gas)
 * US915, Sub-band 2 (channels 8-15 + 500kHz channel 65)
 *
 * Hardware notes:
 * - RAK1901 uses a Sensirion SHTC3 chip (NOT an SHT31). Requires the
 *   SparkFun SHTC3 Arduino Library.
 * - RAK12004 uses an onboard I2C ADC (ADC121C021) at address 0x51, NOT an
 *   analog WisBlock pin. It has a power-enable pin (WB_IO6) that MUST be
 *   driven HIGH before the sensor will respond on I2C - if that pin is
 *   never set, the module stays powered off and "not found" errors will
 *   persist regardless of address. This matches RAK's own official
 *   RAK12004_MQ2_Sampling.ino example.
 *
 * Methane note: the MQ-2 has no official Winsen sensitivity curve for
 * methane. This code approximates methane using the same LPG-style
 * regression constants/method as RAK's own example (log-log model),
 * which gives a directionally useful but not certified methane ppm.
 * For accurate methane ppm, use an MQ-4 instead.
 *
 * Temp/humidity compensation note: this correction is currently DISABLED.
 * The RAK1901 sits close enough to the RAK12004's own heater that it reads
 * meaningfully hotter (~5-10F / ~3-6C, confirmed against a separate
 * reference thermometer) than true room ambient, which made the correction
 * actively wrong rather than helpful. Temp/humidity are still read and
 * reported in the payload for their own sake, just no longer applied to
 * the ppm math. See the commented-out lines in pollSensors() to re-enable
 * if the sensors are ever relocated further apart. When it was enabled,
 * this was always a community approximation anchored at 20C/65%RH = 1.0,
 * not an exact digitization of Winsen's datasheet graph.
 */

#include <Wire.h>
#include <SparkFun_SHTC3.h>        // RAK1901 - SHTC3 library
#include <ADC121C021.h>            // RAK-MQx library - RAK12004 MQ-2

// ---------- Config ----------
#define LORAWAN_APP_PORT      2   // sensor data uplinks
#define LORAWAN_CONFIG_PORT  10   // downlink config port (see receiveCallback)

// --- Runtime-adjustable settings (changed via downlink, see receiveCallback) ---
// Defaults: poll every 60s, uplink heartbeat every 10 min unless an alert
// fires, in which case it uplinks immediately. Mains-powered + Class C
// assumed (see setup()), so there's no battery-life pressure driving these
// numbers - they're set for reasonable alert latency vs. airtime/fair-use.
#define POLL_INTERVAL_MIN_MS      5000UL     // hard floor, prevents a bad downlink from hammering the sensor
#define POLL_INTERVAL_MAX_MS   3600000UL     // 1 hour hard ceiling
#define UPLINK_INTERVAL_MIN_MS   30000UL     // hard floor, protects LoRaWAN airtime/fair-use
#define UPLINK_INTERVAL_MAX_MS 86400000UL    // 24 hour hard ceiling
#define REJOIN_RETRY_INTERVAL_MS  60000UL    // retry join() this often while not joined (e.g. after a gateway outage)

uint32_t g_pollIntervalMs   = 60000;     // 60s default
uint32_t g_uplinkIntervalMs = 600000;    // 10 min default heartbeat

#define MQ2_EN_PIN    WB_IO6   // Logic high enables the RAK12004 - MUST be set before begin()
#define MQ2_ADDRESS   0x51     // Confirmed RAK12004 I2C address per RAK's own example

#define MQ2_RL_KOHM          10.0   // Board load resistor per RAK's example
#define MQ2_CLEAN_AIR_RATIO   1.0   // Rs/R0 in clean air, per RAK's example (NOT the generic 9.83 used by other MQ libraries)

float g_alertThresholdPpm = 2000.0; // ~10% LEL for butane/propane (LEL ~18,000-21,000ppm),
                                     // the gases this sensor's curve is actually calibrated
                                     // against. Comfortably above the MQ-2's 300ppm validated
                                     // floor and above expected nuisance sources (cooking
                                     // fumes, alcohol sanitizer), while still well below the
                                     // sensor's ~10,000ppm ceiling. Treat this as an early-
                                     // warning tier, not a replacement for a certified
                                     // detector's alarm point (typically ~25% LEL). Adjust via
                                     // downlink once real baseline field data is available.

// Regression constants from RAK's own example (LPG-style curve, used here
// as a methane approximation - see header note)
// Regression method 0: PPM = pow(10, (log10(ratio) - B) / A)
#define MQ2_CURVE_A   -0.890
#define MQ2_CURVE_B    1.125

// --- Calibration timing/stability tuning ---
// MQ-2 heater needs real time to reach stable operating temperature after
// power-up - 20 seconds is not enough. 5 minutes matches the routine
// (non-first-use) warm-up window commonly recommended for this sensor.
#define MQ2_WARMUP_MS               300000  // 5 min warm-up before calibration sampling starts
#define MQ2_CAL_SAMPLE_INTERVAL_MS    3000  // time between calibration samples
#define MQ2_CAL_WINDOW_SIZE             10  // rolling window size used for the stability check
#define MQ2_CAL_STABILITY_THRESHOLD   0.05  // accept once stddev/mean of the window is under 5%
#define MQ2_CAL_MAX_DURATION_MS     300000  // give up if it never stabilizes within 5 min of sampling

SHTC3 g_shtc3;
ADC121C021 mq2;

bool shtc3_ok = false;
bool mq2_ok = false;
bool mq2_calibration_stable = false;

// --- Persisted R0 storage (flash) ---
// Ordinary boots load a previously-validated R0 from flash instead of
// blindly recalibrating - this avoids the device silently calibrating
// against contaminated/non-clean air if it happens to reboot during or
// shortly after a real gas event or heavy kitchen activity. Recalibration
// only happens on genuine first-use (no valid data in flash yet) or when
// explicitly requested via the downlink "recalibrate" flag - at which
// point YOU are responsible for making sure the air is actually clean
// before triggering it.
//
// History: an earlier version of this write (offset 0x0000) coincided
// with the device's default BLE advertising breaking (stopped appearing
// in WisToolBox). BLE was restored by explicitly calling
// api.ble.settings.blemode(RAK_BLE_UART_MODE) + api.ble.uart.start() in
// setup() (see below) rather than relying on RUI3's default auto-start
// behavior - notably, BLE came back WITHOUT reverting this flash write or
// erasing anything, which is decent evidence this offset itself wasn't
// the actual cause. Not airtight proof, but reasonable grounds to
// re-enable this. If BLE issues recur, this offset is the first thing to
// suspect again.
#define R0_FLASH_OFFSET   0x0000
#define R0_FLASH_MAGIC    0x52304D32UL  // "R0M2" - identifies valid stored data, distinguishes from blank/erased flash

struct R0FlashRecord
{
    uint32_t magic;
    float r0;
};

bool saveR0ToFlash(float r0)
{
    R0FlashRecord rec;
    rec.magic = R0_FLASH_MAGIC;
    rec.r0 = r0;

    bool ok = api.system.flash.set(R0_FLASH_OFFSET, (uint8_t *)&rec, sizeof(rec));
    Serial.printf("Flash save R0=%.2f: %s\n", r0, ok ? "OK" : "FAILED");
    return ok;
}

// Returns true and fills *outR0 if a valid stored calibration was found.
bool loadR0FromFlash(float *outR0)
{
    R0FlashRecord rec;
    bool ok = api.system.flash.get(R0_FLASH_OFFSET, (uint8_t *)&rec, sizeof(rec));

    if (!ok || rec.magic != R0_FLASH_MAGIC || isnan(rec.r0) || isinf(rec.r0) || rec.r0 <= 0)
    {
        return false; // no valid record yet (first boot) or corrupted
    }

    *outR0 = rec.r0;
    return true;
}

// --- Latest cached sensor reading, updated by pollSensors(), sent by buildAndSendPayload() ---
float g_lastTemp = 20.0;
float g_lastHum = 65.0;
float g_lastMethanePPM = 0;
bool g_lastAlert = false;

// Set true for the duration of calibrateMQ2(), false otherwise. The tick
// scheduler checks this and skips polling/sending entirely while it's set.
// This matters because RUI3's periodic timer callback (tickCallback) runs
// concurrently with a blocking delay() elsewhere (e.g. inside a downlink's
// receiveCallback) rather than being blocked by it - without this guard,
// pollSensors() would call mq2.readSensor() on the same I2C bus/address
// while calibrateMQ2() is mid-sampling, risking corrupted I2C transactions
// on both sides. This was NOT an issue for the calibration that runs during
// setup(), since nothing else is running yet at that point - it only
// matters for a recalibration triggered later via downlink, after the tick
// timer is already active.
volatile bool mq2_calibrating = false;

// ---------- Temp/Humidity correction (approximation, see header note) ----------
// Returns a multiplier applied to the final ppm reading.
float getTempHumidityCorrection(float tempC, float humidityRH)
{
    if (tempC < -20) tempC = -20;
    if (tempC > 50) tempC = 50;
    if (humidityRH < 15) humidityRH = 15;
    if (humidityRH > 95) humidityRH = 95;

    float tempFactor = 1.0 - ((tempC - 20.0) * 0.0032);
    float humFactor  = 1.0 - ((humidityRH - 65.0) * 0.0022);

    float correction = tempFactor * humFactor;

    if (correction < 0.5) correction = 0.5;
    if (correction > 1.5) correction = 1.5;

    return correction;
}

// ---------- MQ-2 calibration (run once at startup, sensor must be in clean air) ----------
// Waits through a real warm-up period, then samples R0 repeatedly and only
// accepts a value once a rolling window of samples has settled to within
// MQ2_CAL_STABILITY_THRESHOLD of its own mean. This catches both a sensor
// that's still thermally settling AND a genuinely flaky component (bad
// pot/wiper contact, marginal connection) that never converges - either
// way, we don't want to silently lock in a bad R0 and report misleading
// ppm data from it afterward.
void calibrateMQ2()
{
    mq2_calibrating = true; // block tickCallback from touching the I2C sensor until this returns

    Serial.printf("MQ-2 warm-up: waiting %lu ms before calibration...\n", (unsigned long)MQ2_WARMUP_MS);
    delay(MQ2_WARMUP_MS);

    Serial.println("Calibrating MQ-2 R0 in clean air (do not do this near gas sources)...");
    mq2.setRL(MQ2_RL_KOHM);

    float window[MQ2_CAL_WINDOW_SIZE];
    int windowCount = 0;
    int windowIndex = 0;

    uint32_t calStart = millis();
    bool stable = false;
    float acceptedR0 = 0;

    while ((millis() - calStart) < MQ2_CAL_MAX_DURATION_MS)
    {
        float sample = mq2.calibrateR0(MQ2_CLEAN_AIR_RATIO);

        if (isinf(sample) || isnan(sample) || sample <= 0)
        {
            Serial.println("Warning: sample is invalid (open circuit / short?) - skipping this sample.");
            delay(MQ2_CAL_SAMPLE_INTERVAL_MS);
            continue;
        }

        window[windowIndex] = sample;
        windowIndex = (windowIndex + 1) % MQ2_CAL_WINDOW_SIZE;
        if (windowCount < MQ2_CAL_WINDOW_SIZE) windowCount++;

        if (windowCount == MQ2_CAL_WINDOW_SIZE)
        {
            float mean = 0;
            for (int i = 0; i < MQ2_CAL_WINDOW_SIZE; i++) mean += window[i];
            mean /= MQ2_CAL_WINDOW_SIZE;

            float variance = 0;
            for (int i = 0; i < MQ2_CAL_WINDOW_SIZE; i++)
            {
                float d = window[i] - mean;
                variance += d * d;
            }
            variance /= MQ2_CAL_WINDOW_SIZE;
            float stddev = sqrt(variance);
            float relativeSpread = (mean > 0) ? (stddev / mean) : 999;

            Serial.printf("  sample R0=%.2f  window mean=%.2f  relative spread=%.1f%%\n",
                          sample, mean, relativeSpread * 100.0);

            if (relativeSpread < MQ2_CAL_STABILITY_THRESHOLD)
            {
                acceptedR0 = mean;
                stable = true;
                break;
            }
        }
        else
        {
            Serial.printf("  sample R0=%.2f  (building window: %d/%d)\n", sample, windowCount, MQ2_CAL_WINDOW_SIZE);
        }

        delay(MQ2_CAL_SAMPLE_INTERVAL_MS);
    }

    mq2_calibration_stable = stable;

    if (!stable)
    {
        Serial.println("Warning: MQ-2 R0 never stabilized within the timeout - check for a flaky "
                        "connection or component (e.g. worn pot wiper). Gas sensor disabled this boot.");
        mq2_ok = false;
        mq2_calibrating = false;
        return;
    }

    mq2.setR0(acceptedR0);
    Serial.printf("MQ-2 R0 calibrated (stable): %.2f\n", acceptedR0);
    saveR0ToFlash(acceptedR0);

    mq2.setRegressionMethod(0); // PPM = pow(10, (log10(ratio)-B)/A), matches RAK's example
    mq2.setA(MQ2_CURVE_A);
    mq2.setB(MQ2_CURVE_B);

    mq2_calibrating = false;
}

// ---------- LoRaWAN callbacks ----------
void joinCallback(int32_t status)
{
    Serial.println(status == 0 ? "LoRaWAN join success" : "LoRaWAN join failed");
}

void sendCallback(int32_t status)
{
    Serial.println(status == 0 ? "Uplink sent successfully" : "Uplink failed");
}

// Single-byte command shortcuts on LORAWAN_CONFIG_PORT - paste the hex
// directly into a downlink queue, no need to build the full 7-byte config
// payload for these common actions.
#define CMD_RECALIBRATE       0x01  // trigger a fresh R0 calibration now (air must be clean)
#define CMD_RESET_DEFAULTS    0x02  // reset poll/uplink/alert threshold to compiled-in defaults

void receiveCallback(SERVICE_LORA_RECEIVE_T *data)
{
    Serial.printf("Downlink received on port %d, size %d\n", data->Port, data->BufferSize);

    if (data->Port != LORAWAN_CONFIG_PORT)
    {
        return; // not a config downlink, ignore
    }

    // --- Short single-byte command form ---
    if (data->BufferSize == 1)
    {
        uint8_t cmd = data->Buffer[0];

        switch (cmd)
        {
        case CMD_RECALIBRATE:
            Serial.println("Command 0x01: recalibrate requested via downlink.");
            if (mq2_ok)
            {
                calibrateMQ2(); // blocking; saves to flash on success (see calibrateMQ2())
            }
            else
            {
                Serial.println("  Ignored: MQ-2 not initialized.");
            }
            break;

        case CMD_RESET_DEFAULTS:
            g_pollIntervalMs = 60000;
            g_uplinkIntervalMs = 600000;
            g_alertThresholdPpm = 2000.0;
            Serial.println("Command 0x02: poll/uplink/alertThreshold reset to compiled-in defaults.");
            break;

        default:
            Serial.printf("Unknown single-byte command: 0x%02X\n", cmd);
            break;
        }
        return;
    }

    // --- Full structured config form (unchanged) ---
    if (data->BufferSize != 7)
    {
        Serial.println("Config downlink ignored: expected either 1 byte (command) or 7 bytes "
                        "[pollSec(2) uplinkSec(2) alertThresholdPpmX10(2) recalibrate(1)]");
        return;
    }

    uint8_t *b = data->Buffer;

    uint16_t pollSec   = (b[0] << 8) | b[1];
    uint16_t uplinkSec = (b[2] << 8) | b[3];
    uint16_t alertPpmX10 = (b[4] << 8) | b[5]; // ppm * 10, e.g. 5000 = 500.0 ppm
    bool recalibrate   = b[6] != 0;

    uint32_t newPollMs = (uint32_t)pollSec * 1000UL;
    uint32_t newUplinkMs = (uint32_t)uplinkSec * 1000UL;

    if (newPollMs < POLL_INTERVAL_MIN_MS) newPollMs = POLL_INTERVAL_MIN_MS;
    if (newPollMs > POLL_INTERVAL_MAX_MS) newPollMs = POLL_INTERVAL_MAX_MS;
    if (newUplinkMs < UPLINK_INTERVAL_MIN_MS) newUplinkMs = UPLINK_INTERVAL_MIN_MS;
    if (newUplinkMs > UPLINK_INTERVAL_MAX_MS) newUplinkMs = UPLINK_INTERVAL_MAX_MS;

    g_pollIntervalMs = newPollMs;
    g_uplinkIntervalMs = newUplinkMs;
    g_alertThresholdPpm = alertPpmX10 / 10.0;

    Serial.printf("Config updated: poll=%lums uplink=%lums alertThreshold=%.1fppm recalibrate=%d\n",
                  (unsigned long)g_pollIntervalMs, (unsigned long)g_uplinkIntervalMs,
                  g_alertThresholdPpm, recalibrate);

    if (recalibrate && mq2_ok)
    {
        Serial.println("Recalibration requested via downlink - running now (blocking).");
        calibrateMQ2();
    }
}

// ---------- Poll sensors and update cached readings (no radio activity) ----------
void pollSensors()
{
    float temperature = 20.0; // safe fallback = reference condition
    float humidity = 65.0;

    if (shtc3_ok)
    {
        SHTC3_Status_TypeDef result = g_shtc3.update();
        if (result == SHTC3_Status_Nominal && g_shtc3.lastStatus == SHTC3_Status_Nominal)
        {
            temperature = g_shtc3.toDegC();
            humidity = g_shtc3.toPercent();
        }
        else
        {
            Serial.println("Failed to read SHTC3, using fallback values");
        }
    }

    float methanePPM = 0;
    bool alertTriggered = false;

    if (mq2_ok)
    {
        float rawPPM = mq2.readSensor(); // library computes ppm using R0/A/B already set

        // Temp/humidity correction is disabled: the RAK1901 sits close enough
        // to the RAK12004's own heater that it reads ~5-10F (~3-6C) hotter
        // than a separate reference thermometer in the same room. Feeding that
        // biased reading into the correction was actively skewing ppm rather
        // than improving it. Temp/humidity are still reported in the payload
        // for their own sake (room monitoring), just no longer used to
        // adjust the gas reading. Re-enable by restoring the two lines below
        // if the sensors are ever relocated further apart and the bias is
        // confirmed gone.
        // float correction = getTempHumidityCorrection(temperature, humidity);
        // methanePPM = rawPPM * correction;
        methanePPM = rawPPM;

        // Alert is evaluated directly against the computed ppm value rather
        // than the ADC121C021's raw-count hardware comparator. Raw counts
        // only mean a fixed ppm level for whatever R0 was in effect when the
        // threshold was set - since R0 changes on every (re)calibration, a
        // raw threshold silently drifts out of meaning over time. Comparing
        // ppm directly stays correct automatically. This does mean the
        // sensor's own hardware alert pin/comparator is no longer the
        // authoritative alert source - g_alertThresholdPpm below is.
        alertTriggered = (methanePPM >= g_alertThresholdPpm);
        if (alertTriggered)
        {
            Serial.printf("MQ-2 ALERT: %.1f ppm >= threshold %.1f ppm\n", methanePPM, g_alertThresholdPpm);
        }
    }

    Serial.printf("Temp: %.2f C  Hum: %.2f %%  CH4(approx): %.1f ppm  Alert: %d\n",
                  temperature, humidity, methanePPM, alertTriggered);

    g_lastTemp = temperature;
    g_lastHum = humidity;
    g_lastMethanePPM = methanePPM;
    g_lastAlert = alertTriggered;
}

// ---------- Build payload from the latest cached reading and transmit ----------
void buildAndSendPayload()
{
    // Format (10 bytes):
    // [0-1] temperature *100 (int16, signed)
    // [2-3] humidity *100 (uint16)
    // [4-5] methane approx ppm *10 (uint16, capped at 6553.5 ppm)
    // [6]   shtc3 status (1 = ok, 0 = fail)
    // [7]   mq2 status (1 = ok, 0 = fail)
    // [8]   mq2 alert flag (1 = threshold violated, 0 = normal)
    // [9]   mq2 calibration stable (1 = R0 settled within tolerance, 0 = never stabilized - treat ppm as unreliable)
    uint8_t payload[10];
    int16_t temp_i = (int16_t)(g_lastTemp * 100);
    uint16_t hum_i = (uint16_t)(g_lastHum * 100);

    float methanePPM = g_lastMethanePPM;
    if (methanePPM < 0) methanePPM = 0;
    if (methanePPM > 6553.5) methanePPM = 6553.5;
    uint16_t ppm_i = (uint16_t)(methanePPM * 10);

    payload[0] = temp_i >> 8;
    payload[1] = temp_i & 0xFF;
    payload[2] = hum_i >> 8;
    payload[3] = hum_i & 0xFF;
    payload[4] = ppm_i >> 8;
    payload[5] = ppm_i & 0xFF;
    payload[6] = shtc3_ok ? 1 : 0;
    payload[7] = mq2_ok ? 1 : 0;
    payload[8] = g_lastAlert ? 1 : 0;
    payload[9] = mq2_calibration_stable ? 1 : 0;

    if (api.lorawan.send(sizeof(payload), payload, LORAWAN_APP_PORT, true))
    {
        Serial.println("Uplink queued");
    }
    else
    {
        Serial.println("Uplink failed to queue (check join status / duty cycle)");
    }
}

// ---------- Scheduler: poll on g_pollIntervalMs, uplink on heartbeat or immediately on alert ----------
void tickCallback(void *data)
{
    static uint32_t lastPollMillis = 0;
    static uint32_t lastUplinkMillis = 0;
    static uint32_t lastJoinAttemptMillis = 0;
    static bool firstRun = true;
    static bool wasAlert = false;

    uint32_t now = millis();

    if (!api.lorawan.njs.get())
    {
        // Not joined (never joined yet, or lost the session after an
        // outage). Actively retry rather than just waiting - a single
        // failed join at boot (e.g. gateway briefly down after a power
        // outage) would otherwise leave the device stuck indefinitely even
        // once the network is reachable again. Retries are throttled to
        // REJOIN_RETRY_INTERVAL_MS rather than every tick, both to avoid
        // spamming join attempts and because sensor polling is paused here
        // too - no point hammering the I2C sensors with nowhere to send.
        if ((now - lastJoinAttemptMillis) >= REJOIN_RETRY_INTERVAL_MS)
        {
            Serial.println("Not joined to LoRaWAN - attempting (re)join...");
            api.lorawan.join();
            lastJoinAttemptMillis = now;
        }
        return;
    }

    if (mq2_calibrating)
    {
        return; // recalibration in progress (see calibrateMQ2()) - don't touch the I2C sensor concurrently
    }

    if (firstRun || (now - lastPollMillis) >= g_pollIntervalMs)
    {
        pollSensors();
        lastPollMillis = now;

        bool alertJustCleared = wasAlert && !g_lastAlert;

        if (g_lastAlert)
        {
            Serial.println("Alert active - sending uplink immediately (not waiting for heartbeat)");
            buildAndSendPayload();
            lastUplinkMillis = now;
        }
        else if (alertJustCleared)
        {
            Serial.println("Alert cleared - sending uplink immediately so downstream state doesn't go stale");
            buildAndSendPayload();
            lastUplinkMillis = now;
        }
        else if (firstRun || (now - lastUplinkMillis) >= g_uplinkIntervalMs)
        {
            buildAndSendPayload();
            lastUplinkMillis = now;
        }

        wasAlert = g_lastAlert;
        firstRun = false;
    }
}

void setup()
{
    Serial.begin(115200);
    time_t serial_timeout = millis();
    while (!Serial && (millis() - serial_timeout) < 5000)
    {
        delay(10);
    }

    Serial.println("RAK4631 + RAK1901 (SHTC3) + RAK12004 (MQ-2) - US915, sub-band 2");

    // --- Explicit BLE UART service start (recovery fix) ---
    // Default BLE advertising broke on this unit after a prior bad flash
    // write (see note above). This matches RAK's own default/factory app
    // pattern (BLE UART mode + api.ble.uart.start()) rather than the
    // generic api.ble.advertise.start(), which didn't restore WisToolBox
    // visibility. Confirmed working.
    Serial6.begin(115200, RAK_AT_MODE);
    api.ble.settings.blemode(RAK_BLE_UART_MODE);
    api.ble.uart.start(0);
    Serial.println("BLE UART start() called");

    // --- I2C init ---
    Wire.begin();

    // --- RAK19007 base board: enable 3V3_S rail for sensor/module slots ---
    // On RAK19007 (unlike RAK5005-O), module slot power is gated behind
    // this switched 3.3V rail, controlled via WB_IO2. Without this, modules
    // in the sensor slots may get no power at all regardless of their own
    // EN pin state.
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, HIGH);
    delay(100);

    // --- RAK1901 (SHTC3) init ---
    if (g_shtc3.begin() != SHTC3_Status_Nominal)
    {
        Serial.println("RAK1901 (SHTC3) not found - check wiring/slot");
        shtc3_ok = false;
    }
    else
    {
        shtc3_ok = true;
    }

    // --- RAK12004 (MQ-2) power-on and init ---
    pinMode(MQ2_EN_PIN, OUTPUT);
    digitalWrite(MQ2_EN_PIN, HIGH); // power on RAK12004 - required before I2C will respond
    delay(500);

    int mq2_retries = 0;
    while (!(mq2_ok = mq2.begin(MQ2_ADDRESS, Wire)) && mq2_retries < 10)
    {
        Serial.println("RAK12004 MQ-2 not found - check EN pin/wiring/slot, retrying...");
        delay(200);
        mq2_retries++;
    }

    if (mq2_ok)
    {
        mq2.setRL(MQ2_RL_KOHM);
        mq2.setRegressionMethod(0); // PPM = pow(10, (log10(ratio)-B)/A), matches RAK's example
        mq2.setA(MQ2_CURVE_A);
        mq2.setB(MQ2_CURVE_B);

        float storedR0;
        if (loadR0FromFlash(&storedR0))
        {
            mq2.setR0(storedR0);
            mq2_calibration_stable = true;
            Serial.printf("Loaded previously-validated R0 from flash: %.2f "
                           "(skipping recalibration - send a downlink with "
                           "recalibrate=1 to force a fresh one in known-clean air)\n",
                           storedR0);
        }
        else
        {
            Serial.println("No valid stored R0 found - first use, running full calibration.");
            calibrateMQ2(); // handles its own warm-up + stability-checked sampling
        }
        // Alerting is handled entirely in software against g_alertThresholdPpm
        // (see pollSensors()) rather than the ADC121C021's hardware
        // comparator, so no comparator/threshold config is needed here -
        // see the note in pollSensors() for why.
    }
    else
    {
        Serial.println("RAK12004 MQ-2 init failed after retries - continuing without gas sensor");
    }

    // --- LoRaWAN setup ---
    api.lorawan.band.set(RAK_REGION_US915);

    // US915 sub-band 2: channels 8-15 + 500kHz channel 65
    uint16_t channelMask = 0x0002;
    api.lorawan.mask.set(&channelMask);

    api.lorawan.deviceClass.set(RAK_LORA_CLASS_C);
    // Class C keeps RX open continuously, so downlink config changes land
    // immediately rather than waiting for the device's next scheduled
    // uplink. This trades continuous receiver power for responsiveness -
    // reasonable now that the unit is mains-powered rather than battery.
    api.lorawan.njm.set(RAK_LORA_OTAA);

    // If credentials aren't already provisioned via AT commands, uncomment
    // and fill in your keys:
    /*
    uint8_t node_device_eui[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t node_app_eui[8]    = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t node_app_key[16]   = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    api.lorawan.deui.set(node_device_eui, 8);
    api.lorawan.appeui.set(node_app_eui, 8);
    api.lorawan.appkey.set(node_app_key, 16);
    */

    api.lorawan.registerRecvCallback(receiveCallback);
    api.lorawan.registerJoinCallback(joinCallback);
    api.lorawan.registerSendCallback(sendCallback);

    if (!api.lorawan.njs.get())
    {
        Serial.println("Joining LoRaWAN network...");
        api.lorawan.join();
    }

    api.system.timer.create(RAK_TIMER_0, tickCallback, RAK_TIMER_PERIODIC);
    api.system.timer.start(RAK_TIMER_0, 1000, NULL); // 1s scheduler tick; actual poll/uplink
                                                       // cadence is governed by g_pollIntervalMs
                                                       // and g_uplinkIntervalMs, not this value
}

void loop()
{
    api.system.sleep.all();
}
