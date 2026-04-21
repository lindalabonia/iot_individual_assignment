#ifndef SAMPLER_H
#define SAMPLER_H

#include <Arduino.h>

// Oversampler: fills FFT buffer at MAX_SAMPLING_FREQ, then notifies FFT task
void TaskOversampler(void *pvParameters);

// Adaptive sampler: reads ADC at the reduced rate, pushes to queue
void TaskAdaptiveSampler(void *pvParameters);

#endif
