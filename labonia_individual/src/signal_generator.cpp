#include "signal_generator.h"
#include "config.h"
#include "globals.h"
#include <Wire.h>
#include <math.h>

// Signal definition: s(t) = sum of a_k * sin(2*pi*f_k*t)
const float amplitudes[NUM_COMPONENTS]  = {5.0, 2.0, 4.0};
const float frequencies[NUM_COMPONENTS] = {1.0, 25.0, 5.0};

// MCP4725 fast-write 
// The DAC value is 12 bits (0-4095) but I2C can only send 8 bits (0-255)
// at a time, so we split it in two:
//   e.g. value = 2048 → binary = 1000 0000 0000 (12 bit)
//        first send:  1000      (top 4 bits)  = 0x08
//        second send: 0000 0000 (low 8 bits)  = 0x00
// Total: 3 bytes on the I2C bus = 1 address (sent automatically by
// beginTransmission) + 2 data bytes. 
void mcp4725_write(uint16_t value) {
    if (value > 4095) value = 4095;
    Wire.beginTransmission(MCP4725_ADDR);  // sends device address (1 byte, automatic)
    Wire.write((value >> 8) & 0x0F);       // top 4 bits
    Wire.write(value & 0xFF);              // low 8 bits
    Wire.endTransmission();
}

// --- Lookup table ---
// Instead of computing sin() at every DAC update, we precompute all the DAC
// values for one full period of the composite signal. At runtime the DAC task
// just cycles through this array. This avoids runtime sin() calls and scales
// to higher frequencies without code changes.
//
// How it works:
//   - The composite signal repeats after T = 1/GCD(f1, f2, ...) seconds.
//     For 3 Hz + 5 Hz: T = 1/GCD(3,5) = 1/1 = 1 second.
//   - The update rate = max_freq * DAC_POINTS_PER_CYCLE (e.g. 5*40 = 200 Hz).
//   - LUT size = update_rate * T (e.g. 200 * 1 = 200 entries).
//   - All of this is computed automatically from the frequencies array.
static uint16_t signalLUT[DAC_LUT_MAX];
static int      lutSize = 0;       // actual number of entries (computed at runtime)
static float    dacUpdateRate = 0; // Hz (computed at runtime)

// Integer GCD — used to find the composite signal period
static int gcd(int a, int b) {
    while (b) { int t = b; b = a % b; a = t; }
    return a;
}

void buildSignalLUT() {
    // Find max frequency for update rate
    float maxFreq = 0;
    for (int k = 0; k < NUM_COMPONENTS; k++)
        if (frequencies[k] > maxFreq) maxFreq = frequencies[k];

    dacUpdateRate = maxFreq * DAC_POINTS_PER_CYCLE;

    // Composite period T = 1 / GCD(all frequencies).
    // We work in integer mHz to avoid floating point GCD issues.
    int gcdFreq = (int)(frequencies[0] * 1000);
    for (int k = 1; k < NUM_COMPONENTS; k++)
        gcdFreq = gcd(gcdFreq, (int)(frequencies[k] * 1000));
    float compositePeriod = 1000.0 / gcdFreq;  // seconds

    lutSize = (int)(dacUpdateRate * compositePeriod);
    if (lutSize > DAC_LUT_MAX) lutSize = DAC_LUT_MAX;

    // Find max amplitude sum for normalization
    float maxAmpSum = 0;
    for (int k = 0; k < NUM_COMPONENTS; k++)
        maxAmpSum += fabs(amplitudes[k]);

    // Precompute DAC values for one full composite period
    for (int i = 0; i < lutSize; i++) {
        float t = (float)i / dacUpdateRate;
        float sig = 0.0;
        for (int k = 0; k < NUM_COMPONENTS; k++)
            sig += amplitudes[k] * sin(2.0 * PI * frequencies[k] * t);

        // Normalize [-maxAmpSum, +maxAmpSum] -> [0, 4095]
        signalLUT[i] = (uint16_t)(DAC_OFFSET + (sig / maxAmpSum) * DAC_AMPLITUDE);
    }

    // Serial.printf("[DAC] LUT built: %d entries, update rate %.0f Hz, period %.3f s\n",
    //               lutSize, dacUpdateRate, compositePeriod);
}

// DAC task: cycles through the precomputed lookup table, writing each value
// to the MCP4725 at the fixed update rate.
void TaskDACGenerator(void *pvParameters) {
    // periodMs = time between consecutive DAC writes in milliseconds.
    // e.g. at 200 Hz update rate: 1000 / 200 = 5 ms (one write every 5 ms)
    // Use vTaskDelay instead of delayMicroseconds so the CPU is released
    // between writes.

    // Caveat: FreeRTOS tick = 1 ms, so periodMs is truncated to an integer.
    // If 1000/dacUpdateRate is not an integer the LUT is consumed faster
    // than intended and every frequency in the signal gets shifted up by
    // the same ratio. E.g. maxFreq=7 Hz → updateRate=280 Hz → 3.57 ms
    // truncated to 3 ms (instead of waiting 3.57 ms between writes, we wait 3 ms)
    // Choose frequencies that keep 1000/(maxFreq*DAC_POINTS_PER_CYCLE)
    // integer, or switch to delayMicroseconds() for sub-ms resolution.
    
    unsigned long periodMs = (unsigned long)(1000.0 / dacUpdateRate);
    if (periodMs < 1) periodMs = 1;  // minimum 1 ms for vTaskDelay

    int idx = 0;
    while (1) {
        mcp4725_write(signalLUT[idx]);
        idx++;
        if (idx >= lutSize) idx = 0;  // wrap around = signal repeats

        vTaskDelay(pdMS_TO_TICKS(periodMs));
    }
}
