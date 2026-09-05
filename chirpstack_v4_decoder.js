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
 * Matches the 5-byte payload produced by the RAK4631 + RAK12004 (MQ-2)
 * sketch, sent on LoRaWAN FPort 2.
 *
 * Temperature/humidity (RAK1901/SHTC3) is no longer transmitted - that
 * module reads far hotter than true ambient regardless of mounting
 * location (board/enclosure heat buildup, not simple proximity to the
 * MQ-2) and isn't trustworthy as room data. A separate Milesight EM320-TH
 * covers room temp/humidity in this deployment instead.
 *
 * Payload layout (5 bytes):
 * [0-1] methane approx ppm * 10 (uint16, big-endian)       -> ppm (LPG-curve approximation, see sketch notes)
 * [2]   mq2_ok              (uint8, 1 = sensor read OK, 0 = fail)
 * [3]   mq2_alert          (uint8, 1 = methane_ppm_approx >= the configured
 *       ppm alert threshold, 0 = normal - evaluated in software each poll,
 *       not by the sensor's hardware comparator)
 * [4]   mq2_calibration_stable (uint8, 1 = R0 settled within tolerance during
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

    if (bytes.length !== 5) {
        return {
            data: {},
            errors: ["Unexpected payload length: " + bytes.length + ", expected 5 bytes"],
        };
    }

    // --- Methane approx ppm (uint16, big-endian) ---
    var ppmRaw = (bytes[0] << 8) | bytes[1];
    var methane_ppm_approx = ppmRaw / 10.0;

    // --- Status flags ---
    var mq2_ok = bytes[2] === 1;
    var mq2_alert = bytes[3] === 1;
    var mq2_calibration_stable = bytes[4] === 1;

    if (!mq2_ok) {
        warnings.push("MQ-2 (RAK12004) reported a failed read on this uplink");
    }
    if (mq2_ok && !mq2_calibration_stable) {
        warnings.push("MQ-2 R0 never stabilized during calibration - treat methane_ppm_approx as unreliable (check for a flaky connection/component)");
    }

    return {
        data: {
            methane_ppm_approx: methane_ppm_approx, // ppm, LPG-curve approximation - see sketch notes
            mq2_ok: mq2_ok,
            mq2_alert: mq2_alert,
            mq2_calibration_stable: mq2_calibration_stable,
        },
        warnings: warnings,
        errors: errors,
    };
}
