#include "fft_processor.h"
#include "config.h"
#include "globals.h"
#include "sampler.h"
#include <arduinoFFT.h>

// Scan magnitude bins to find the HIGHEST frequency with significant energy.
// majorPeak() only gives the loudest frequency — we need the highest one
// because Nyquist requires f_sample >= 2 * f_max.
//
// Example: signal = 4*sin(2pi*3t) + 2*sin(2pi*5t)
//   majorPeak() -> 3 Hz (wrong for our purpose)
//   findMaxFrequency() -> 5 Hz (correct)
//
// The threshold is relative: FFT_MAG_THRESHOLD_RATIO * peak magnitude.
// This separates real signal components from noise/spectral leakage
// automatically, regardless of signal amplitude or ADC configuration.
float findMaxFrequency(double *magnitudes, int samples, float samplingFreq) {
    // Find peak magnitude for noise floor threshold (skip bin 0 = DC)
    double peakMag = 0.0;
    for (int i = 1; i < samples / 2; i++)
        if (magnitudes[i] > peakMag) peakMag = magnitudes[i];

    double threshold = peakMag * FFT_MAG_THRESHOLD_RATIO;
    float freqRes = samplingFreq / (float)samples;

    // Highest-frequency LOCAL PEAK above threshold. Local peak = bin higher
    // than both neighbors. Filters out spectral leakage (which decreases
    // monotonically away from a real peak, so leakage bins are never local maxima).
    float maxFreq = 0.0;
    for (int i = 2; i < samples / 2 - 1; i++) {
        if (magnitudes[i] > threshold &&
            magnitudes[i] > magnitudes[i - 1] &&
            magnitudes[i] > magnitudes[i + 1]) {
            float freq = i * freqRes;
            if (freq > maxFreq) maxFreq = freq;
        }
    }
    return maxFreq;
}

// In-place FFT pipeline. Uses a static imag buffer (zeroed each call) so
// callers don't have to provide one. Destroys `buf` (writes magnitudes back).
float runFFTAndFindMax(double *buf, int n, float samplingFreq) {
    static double imag[FFT_SAMPLES];
    for (int i = 0; i < n; i++) imag[i] = 0.0;

    // DC removal: ADC mid-scale (~2048) would dominate bin 0 and skew the
    // relative threshold inside findMaxFrequency.
    double mean = 0.0;
    for (int i = 0; i < n; i++) mean += buf[i];
    mean /= n;
    for (int i = 0; i < n; i++) buf[i] -= mean;

    ArduinoFFT<double> FFT(buf, imag, n, samplingFreq);

    // Hamming window reduces spectral leakage
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    return findMaxFrequency(buf, n, samplingFreq);
}

// Set the adaptive rate from a detected f_max and start the steady-state
// sampler.
void applyAdaptiveRate(float maxFreq) {
    // Nyquist: f_sample > 2*f_max. 2.2x adds ~10% margin to absorb the FFT
    // bin resolution error (e.g. real 5 Hz detected as 4.88 Hz at 0.977 Hz/bin).
    if (maxFreq > 0) {
        detectedMaxFreq = maxFreq;
        adaptiveSamplingFreq = maxFreq * 2.2;

        Serial.printf("[FFT] Detected max frequency: %.2f Hz\n", maxFreq);
        Serial.printf("[FFT] Adaptive rate: %.1f Hz (%.0f -> %.1f, %.1fx reduction)\n",
                       adaptiveSamplingFreq, MAX_SAMPLING_FREQ, adaptiveSamplingFreq,
                       MAX_SAMPLING_FREQ / adaptiveSamplingFreq);
    }
    fftDone = true;

    xTaskCreatePinnedToCore(TaskAdaptiveSampler, "AdaptSamp",
                            4096, NULL, 2, &adaptiveTaskHandle, 1);
}

// Runs ONCE at startup. Waits for the oversampler buffer, computes FFT,
// determines f_max, sets the adaptive sampling rate, then spawns the
// adaptive sampler task and deletes itself.
//
// One-shot: signal is a fixed sum of sinusoids → frequency content stationary.
// In BONUS mode this task is replaced by TaskFilter, which does the same job
// after injecting noise/anomalies and evaluating filters.
void TaskFFTProcessor(void *pvParameters) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float maxFreq = runFFTAndFindMax(vReal, FFT_SAMPLES, MAX_SAMPLING_FREQ);
    applyAdaptiveRate(maxFreq);

    vTaskDelete(NULL);
}
