#ifndef FFT_PROCESSOR_H
#define FFT_PROCESSOR_H

#include <Arduino.h>

// Scan all FFT bins and return the highest frequency above threshold.
// WARNING: do NOT use majorPeak() for this — it returns the dominant
// frequency, not the highest one present.
float findMaxFrequency(double *magnitudes, int samples, float samplingFreq);

// In-place FFT pipeline on the given buffer: DC removal, Hamming window,
// FFT, magnitude, then findMaxFrequency. Buffer is destroyed (replaced
// with magnitudes). Caller must pass copies if the original is needed.
float runFFTAndFindMax(double *buf, int n, float samplingFreq);

// Apply detected f_max to the runtime: set adaptiveSamplingFreq, mark fftDone,
// spawn TaskAdaptiveSampler. Shared by FFT task and filter task.
void applyAdaptiveRate(float maxFreq);

// FFT task: waits for buffer, computes FFT, sets adaptive rate, spawns sampler
void TaskFFTProcessor(void *pvParameters);

#endif
