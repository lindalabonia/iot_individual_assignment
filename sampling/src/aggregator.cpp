#include "aggregator.h"
#include "config.h"
#include "globals.h"

#if !defined(USE_LORAWAN) && !defined(SKIP_TX)
#include "wifi_mqtt.h"
#endif

// Collects samples from the queue and computes the average over each window.
// Uses a FIXED (tumbling) window: collects exactly samplesPerWindow samples,
// computes the average, outputs the result, then starts a fresh window with
// sum and count reset to zero. No overlap between consecutive windows.
void TaskAggregator(void *pvParameters) {
    // Wait until FFT has determined the adaptive frequency
    while (!fftDone)
        vTaskDelay(pdMS_TO_TICKS(100));

    int samplesPerWindow = (int)(adaptiveSamplingFreq * WINDOW_DURATION_SEC);
    // Serial.printf("[AGG] Window: %d s, expected samples: %d\n",
    //               WINDOW_DURATION_SEC, samplesPerWindow);

    int windowNum = 0;

    while (1) {
        float sum = 0.0;
        int count = 0;
        unsigned long t0 = millis();

        while (count < samplesPerWindow) {
            float sample;
            // xQueueReceive blocks until a sample arrives in the queue.
            // The 2000 ms timeout is a safety net: if the adaptive sampler
            // stops producing samples for some reason, we don't hang forever.
            // Under normal operation the timeout never triggers.
            if (xQueueReceive(sampleQueue, &sample, pdMS_TO_TICKS(2000)) == pdTRUE) {
                sum += sample;
                count++;
            }
        }

        unsigned long dt = millis() - t0;
        float avgV = (sum / count / 4095.0) * 3.3;  // raw ADC -> voltage
        windowNum++;

        // Verbose per-window log — commented out for clean latency demo.
        // Re-enable to debug sampling rate / queue behaviour.
        // Serial.printf("[AGG] Window #%d: %d samples in %lu ms, avg=%.3f V\n",
        //               windowNum, count, dt, avgV);

#ifndef SKIP_TX
  #ifdef USE_LORAWAN
        // LoRa sendReceive() blocks for ~7s (TX + RX1@5s + RX2@6s on TTN).
        // Calling it here would stall the aggregator → samples pile up → queue overflow.
        // So LoRa runs in a separate task, and we hand off data via txQueue.
        // xQueueOverwrite on size-1 queue: if LoRa is still busy, the old value is
        // replaced with the fresh one — no backlog, no stale data, no blocking.
        TxFrame_t frame = { avgV, windowNum };
        xQueueOverwrite(txQueue, &frame);
  #else
        // MQTT publish takes ~1-5ms on local WiFi — fast enough to call directly
        // without a separate task or queue.
        mqttPublish(avgV, windowNum);
  #endif
#endif
    }
}
