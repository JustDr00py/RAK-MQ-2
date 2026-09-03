/**
 * ChirpStack v4 encodeDownlink function
 * Builds the downlink for the sketch's receiveCallback(), sent on FPort 10.
 *
 * SHORT COMMAND FORM (recommended for one-off actions):
 * Paste a single hex byte directly into ChirpStack's downlink queue
 * ("raw" / hex payload option) instead of using this JS encoder at all:
 *   01  -> recalibrate now (air must be genuinely clean when you send this)
 *   02  -> reset poll/uplink/alertThreshold to compiled-in defaults
 *
 * FULL CONFIG FORM (this encoder, for setting specific values):
 * Input object (input.data):
 * {
 *   "pollIntervalSec":   60,     // how often the device reads sensors locally
 *   "uplinkIntervalSec": 600,    // heartbeat uplink interval when no alert
 *   "alertThresholdPpm": 2000,   // methane_ppm_approx level that triggers an
 *                                // immediate alert uplink. ~10% LEL for
 *                                // butane/propane (this sensor's curve is
 *                                // calibrated against those, not methane).
 *                                // Keep well above ~300ppm - the MQ-2's
 *                                // validated detection floor - since
 *                                // readings below that aren't reliable.
 *   "recalibrate":       false   // true = trigger a fresh R0 calibration now
 * }
 */

function encodeDownlink(input) {
    var d = input.data;

    var pollSec = d.pollIntervalSec !== undefined ? d.pollIntervalSec : 60;
    var uplinkSec = d.uplinkIntervalSec !== undefined ? d.uplinkIntervalSec : 600;
    var alertPpm = d.alertThresholdPpm !== undefined ? d.alertThresholdPpm : 2000;
    var recalibrate = d.recalibrate ? 1 : 0;

    // Clamp to uint16 range defensively - the device itself also clamps
    // poll/uplink to its own min/max, this just avoids sending obviously
    // malformed bytes.
    pollSec = Math.max(0, Math.min(65535, pollSec));
    uplinkSec = Math.max(0, Math.min(65535, uplinkSec));
    var alertPpmX10 = Math.max(0, Math.min(65535, Math.round(alertPpm * 10)));

    return {
        bytes: [
            (pollSec >> 8) & 0xFF, pollSec & 0xFF,
            (uplinkSec >> 8) & 0xFF, uplinkSec & 0xFF,
            (alertPpmX10 >> 8) & 0xFF, alertPpmX10 & 0xFF,
            recalibrate,
        ],
        fPort: 10,
    };
}

/**
 * ChirpStack v4 decodeUplink function
 * Matches the 10-byte payload produced by the RAK4631 + RAK1901 (SHTC3) +
 * RAK12004 (MQ-2) sketch, sent on LoRaWAN FPort 2.
 *
 * Payload layout (10 bytes):
 * [0-1] temperature * 100   (int16, signed, big-endian)   -> deg C
 * [2-3] humidity * 100      (uint16, big-endian)           -> % RH
 * [4-5] methane approx ppm * 10 (uint16, big-endian)       -> ppm (LPG-curve approximation, see sketch notes)
 * [6]   shtc3_ok            (uint8, 1 = sensor read OK, 0 = fail)
 * [7]   mq2_ok              (uint8, 1 = sensor read OK, 0 = fail)
 * [8]   mq2_alert          (uint8, 1 = methane_ppm_approx >= the configured
 *       ppm alert threshold, 0 = normal - evaluated in software each poll,
 *       not by the sensor's hardware comparator)
 * [9]   mq2_calibration_stable (uint8, 1 = R0 settled within tolerance during
 *       startup calibration, 0 = never stabilized - ppm should be treated as
 *       unreliable/ignored when this is 0, even if mq2_ok is 1)
 */

function decodeUplink(input) {
    var bytes = input.bytes;
    var warnings = [];
    var errors = [];

    if (input.fPort !== 2) {
        return {
            data: {},
            warnings: ["Unexpected fPort: " + input.fPort + ", expected 2"],
        };
    }

    if (bytes.length !== 10) {
        return {
            data: {},
            errors: ["Unexpected payload length: " + bytes.length + ", expected 10 bytes"],
        };
    }

    // --- Temperature (int16, signed, big-endian) ---
    var tempRaw = (bytes[0] << 8) | bytes[1];
    if (tempRaw & 0x8000) {
        tempRaw = tempRaw - 0x10000; // sign-extend to negative
    }
    var temperature = tempRaw / 100.0;

    // --- Humidity (uint16, big-endian) ---
    var humRaw = (bytes[2] << 8) | bytes[3];
    var humidity = humRaw / 100.0;

    // --- Methane approx ppm (uint16, big-endian) ---
    var ppmRaw = (bytes[4] << 8) | bytes[5];
    var methane_ppm_approx = ppmRaw / 10.0;

    // --- Status flags ---
    var shtc3_ok = bytes[6] === 1;
    var mq2_ok = bytes[7] === 1;
    var mq2_alert = bytes[8] === 1;
    var mq2_calibration_stable = bytes[9] === 1;

    if (!shtc3_ok) {
        warnings.push("SHTC3 (RAK1901) reported a failed read on this uplink");
    }
    if (!mq2_ok) {
        warnings.push("MQ-2 (RAK12004) reported a failed read on this uplink");
    }
    if (mq2_ok && !mq2_calibration_stable) {
        warnings.push("MQ-2 R0 never stabilized during calibration - treat methane_ppm_approx as unreliable (check for a flaky connection/component)");
    }

    return {
        data: {
            temperature: temperature,       // deg C
            humidity: humidity,             // % RH
            methane_ppm_approx: methane_ppm_approx, // ppm, LPG-curve approximation - see sketch notes
            shtc3_ok: shtc3_ok,
            mq2_ok: mq2_ok,
            mq2_alert: mq2_alert,
            mq2_calibration_stable: mq2_calibration_stable,
        },
        warnings: warnings,
        errors: errors,
    };
}
