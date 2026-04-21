#include "sampler.h"
#include "config.h"
#include "globals.h"

// Fills the FFT buffer by reading ADC at MAX_SAMPLING_FREQ.
// Runs once at startup to capture the signal spectrum, then notifies
// the FFT task and deletes itself. One-shot because the signal is a
// fixed sum of sinusoids — the frequency content does not change.
void TaskOversampler(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(1000));  // let DAC stabilize

    // periodUs = time between consecutive samples in microseconds.
    // e.g. at 500 Hz: 1000000 / 500 = 2000 us (one sample every 2 ms)
    unsigned long periodUs = (unsigned long)(1000000.0 / MAX_SAMPLING_FREQ);
    unsigned long nextSample = micros();

    for (int i = 0; i < FFT_SAMPLES; i++) {
        // Busy-wait instead of vTaskDelay because vTaskDelay has a minimum
        // granularity of 1 tick = 1 ms. At 500 Hz the period is 2 ms, so
        // ±1 ms of jitter would mean up to 50% timing error per sample,
        // which would corrupt the FFT frequency analysis.
        while (micros() < nextSample) { /* busy-wait */ }
        vReal[i] = (double)analogRead(ADC_PIN);
        vImag[i] = 0;
        nextSample += periodUs;
    }

    // Serial.printf("[SAMPLER] Buffer ready (%d samples at %.0f Hz)\n",
    //               FFT_SAMPLES, MAX_SAMPLING_FREQ);

    // Notify the next stage. In BONUS mode the filter task pre-processes
    // the buffer (noise+anomaly injection, Z-score/Hampel evaluation) and
    // then runs the FFT itself; otherwise the FFT task is woken directly.
#ifdef BONUS
    xTaskNotifyGive(filterTaskHandle);
#else
    xTaskNotifyGive(fftTaskHandle);
#endif
    vTaskDelete(NULL);
}

// Reads ADC at the adaptive frequency and sends samples to the aggregator queue.
// Created dynamically by the FFT task once the optimal rate is known.
// Runs forever — the adaptive rate is fixed after the initial FFT analysis.
void TaskAdaptiveSampler(void *pvParameters) {
    // periodUs = time between consecutive samples in microseconds.
    // e.g. at 12.5 Hz: 1000000 / 12.5 = 80000 us (one sample every 80 ms)
    unsigned long periodUs = (unsigned long)(1000000.0 / adaptiveSamplingFreq);

    // Serial.printf("[ADAPTIVE] Sampling at %.1f Hz\n", adaptiveSamplingFreq);

    while (1) {
        // Low frequencies (period > 10 ms, e.g. our 80 ms at 12.5 Hz):
        // use vTaskDelay to release the CPU. ±1 ms jitter on 80 ms is ~1.2%
        // error — acceptable for aggregation. The IDLE task on Core 1 can
        // run during the delay (FreeRTOS housekeeping like freeing memory
        // of deleted tasks).
        //
        // High frequencies (period ≤ 10 ms): busy-wait for timing accuracy.
        // ±1 ms jitter from vTaskDelay would be >10% of the period.
        if (periodUs > 10000) {
            vTaskDelay(pdMS_TO_TICKS(periodUs / 1000));
        } else {
            unsigned long target = micros() + periodUs;
            while (micros() < target) { /* busy-wait */ }
        }

        float val = (float)analogRead(ADC_PIN);
        xQueueSend(sampleQueue, &val, 0);  // drop if full

        // Plot reconstructed signal on Serial Plotter 
       //Serial.printf(">signal:%.4f\r\n", val * 3.3f / 4095.0f);
    }
}
