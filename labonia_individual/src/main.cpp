// IoT Adaptive Sampling System
// Heltec WiFi LoRa 32 V3 + MCP4725 DAC (loopback: DAC out -> ADC in)
//
// config.h toggles:
//   SKIP_FFT     → oversampling at MAX_SAMPLING_FREQ (no FFT)
//   SKIP_TX      → no transmission (serial only, for energy baseline)
//   USE_LORAWAN  → LoRaWAN + TTN (otherwise WiFi + MQTT)
//
// Startup flow (one-shot):
//   oversample at MAX_FREQ -> FFT finds f_max -> set adaptive rate
// Steady-state flow (loops forever):
//   adaptive sample at 2.2*f_max -> aggregator computes window average
//
// The FFT runs once because the signal is a fixed sum of sinusoids — its
// frequency content does not change. For the bonus (noisy signals), the
// assignment asks to compare FFT on unfiltered vs filtered signals, not
// to re-run FFT periodically.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "globals.h"
#include "signal_generator.h"
#include "sampler.h"
#include "fft_processor.h"
#include "aggregator.h"
#ifdef BONUS
#include "filter.h"
#endif

#if defined(USE_LORAWAN) && !defined(SKIP_TX)
#include "lora_tx.h"
#elif !defined(SKIP_TX)
#include "wifi_mqtt.h"
#endif



void setup() {
    Serial.begin(115200);
    delay(3000); // wait for USB to stabilize
    // Serial.println("BOOT OK");
    // Serial.println("\n=== IoT Adaptive Sampling System ===\n");


    
    // 1. TRANSMISSION INIT
    // SKIP_TX → no radio, serial output only (for energy baseline measurements).
    // USE_LORAWAN → LoRaWAN OTAA join. Otherwise → WiFi + MQTT persistent connection.
#ifdef SKIP_TX
    // Serial.println("[MODE] No transmission (SKIP_TX), serial output only");
#elif defined(USE_LORAWAN)
    setupLoRa();
#else
    // WiFi + MQTT — persistent connection, established once at boot.
    // Reconnecting each time costs more: 2-5s of radio TX at 200-300 mA.
    // Persistent connection costs ~80 mA idle but publishes instantly.
    setupWiFi();
    setupMQTT();
#endif

    // 2. HARDWARE INIT (I2C, ADC, DAC)
    Wire.begin(I2C_SDA, I2C_SCL);

    /*
     Per ogni byte: 8 bit dati + 1 bit ACK = 9 bit
    3 byte(see signal_generator.cpp): 9 × 3 = 27 bit
    + 1 bit START + 1 bit STOP = 29 bit totali

    29 bit / 400000 bit/s = 72.5 µs ≈ ~60-70 µs

    400000 bit/s ÷ 29 bit/scrittura ≈ 13800 scritture/s max

    con 40 points_per_cycle e max_freq 5 Hz → 200 scritture/s, ben al di sotto del limite teorico di 13800 scritture/s,
    quindi non dovrebbero esserci problemi di I2C bottleneck.
    */

    // Each DAC write sends 3 bytes = ~29 bits on the wire.
    // At 100 kHz: ~240 us per write. At 400 kHz: ~70 us per write.
    // We use 400 kHz (fast mode) because the MCP4725 supports it and our
    // jumper wires are short — 100 kHz would also work at our current update
    // rate (200 Hz) but 400 kHz leaves headroom for higher signal frequencies.
    Wire.setClock(400000);

    Wire.beginTransmission(MCP4725_ADDR);
    if (Wire.endTransmission() != 0) {
        // Serial.println("ERROR: MCP4725 not found! Check wiring (SDA=41, SCL=42)");
        while (1) delay(1000);
    }
    // Serial.println("MCP4725 found at 0x60");

    analogSetAttenuation(ADC_11db);  // 0-3.3V range
    analogReadResolution(12);        // 12-bit

    // Precompute lookup table for the DAC signal (sin() computed once, not at runtime)
    buildSignalLUT();

    // Serial.printf("Signal: ");
    // for (int k = 0; k < NUM_COMPONENTS; k++) {
    //     Serial.printf("%.1f*sin(2pi*%.1ft)", amplitudes[k], frequencies[k]);
    //     if (k < NUM_COMPONENTS - 1) Serial.printf(" + ");
    // }
    // Serial.printf("\nOversampling: %.0f Hz, FFT bins: %d\n\n", MAX_SAMPLING_FREQ, FFT_SAMPLES);

    // 3. QUEUES

    // sampleQueue: FIFO between AdaptiveSampler (producer) and Aggregator (consumer).
    // Sized for one full window at max rate, capped at 2000 to limit RAM (~8 KB).
    // After FFT reduces the rate (e.g. to 10.7 Hz), only ~107 samples per window
    // are actually needed, so 2000 is more than enough.
    int qSize = min((int)(MAX_SAMPLING_FREQ * WINDOW_DURATION_SEC), 2000);
    sampleQueue = xQueueCreate(qSize, sizeof(float));

    // txQueue (LoRa only): size=1 with xQueueOverwrite. Aggregator always succeeds
    // (never blocks), LoRaTX always gets the most recent average. If LoRa is still
    // busy, the old value is replaced — fresh data over stale backlog.
    // Contrast with sampleQueue (size 2000): every sample matters for the average.
#if defined(USE_LORAWAN) && !defined(SKIP_TX)
    txQueue = xQueueCreate(1, sizeof(TxFrame_t));
#endif

    // 4. TASKS

    // Core 0: DAC generator + transmission
    //   DAC (pri 2) uses vTaskDelay between writes → frees Core 0 for TX.
    //   WiFi mode: WiFi stack runs on Core 0 by default (no explicit task).
    //   LoRa mode: TaskLoRaTX (pri 1) runs on Core 0. sendReceive() blocks for
    //   ~7s (TX + RX windows), but DAC preempts it thanks to higher priority.
    // Core 1: sampling + processing pipeline
    //   All ADC/FFT tasks on same core to avoid cross-core cache contention.
    //   Oversampler and FFT are one-shot; AdaptiveSampler and Aggregator loop forever.
    xTaskCreatePinnedToCore(
        TaskDACGenerator,  // puntatore alla funzione da eseguire
        "DAC",             // nome (solo per debug)
        4096,              // stack size in bytes (RAM privata del task)
        NULL,              // parametro passato al task (pvParameters)
        2,                 // priorità (2 = alta, 1 = bassa, 0 = idle)
        NULL,              // handle (puntatore per controllare il task dopo)
        0                  // core: 0 o 1 (ESP32-S3 è dual-core)
    );

#ifndef SKIP_FFT
    // Adaptive: oversample → FFT → find f_max → create AdaptiveSampler at 2.2*f_max.
    // In BONUS mode the FFT task is replaced by TaskFilter, which injects
    // noise/anomalies, evaluates Z-score and Hampel, then runs the FFT itself.
    xTaskCreatePinnedToCore(TaskOversampler,  "Sample",  8192, NULL, 2, NULL,            1);
  #ifdef BONUS
    xTaskCreatePinnedToCore(TaskFilter,       "Filter", 16384, NULL, 1, &filterTaskHandle, 1);
  #else
    xTaskCreatePinnedToCore(TaskFFTProcessor, "FFT",     8192, NULL, 1, &fftTaskHandle,  1);
  #endif
    // TaskAdaptiveSampler is created inside FFT/Filter task after f_max is known
#else
    // Oversampling: skip FFT, sample at MAX_SAMPLING_FREQ.
    // adaptiveSamplingFreq defaults to MAX_SAMPLING_FREQ (set in globals.cpp).
    fftDone = true;
    xTaskCreatePinnedToCore(TaskAdaptiveSampler, "AdaptSamp", 4096, NULL, 2, &adaptiveTaskHandle, 1);
    // Serial.printf("[MODE] Oversampling at %.0f Hz (FFT skipped)\n", MAX_SAMPLING_FREQ);
#endif

    xTaskCreatePinnedToCore(TaskAggregator, "Agg", 4096, NULL, 1, NULL, 1);

#if defined(USE_LORAWAN) && !defined(SKIP_TX)
    // LoRa TX on Core 0, pri 1 (below DAC pri 2).
    // SX1262 uses SPI, DAC uses I2C — no bus conflicts on the same core.
    xTaskCreatePinnedToCore(TaskLoRaTX, "LoRaTX", 8192, NULL, 1, NULL, 0);
#endif

    /*
    Lo scheduler FreeRTOS esegue il task a priorità più alta su ogni core.
    Se due task sullo stesso core hanno uguale priorità, si alternano (time-slicing).

    WiFi mode:                          LoRaWAN mode:
        Core 0                              Core 0
    ──────────────────────              ──────────────────────
    TaskDACGenerator (pri 2)            TaskDACGenerator (pri 2)
      vTaskDelay releases CPU             vTaskDelay releases CPU
      for WiFi stack (background)         for TaskLoRaTX (pri 1)

        Core 1                              Core 1
    ──────────────────────              ──────────────────────
    TaskOversampler (pri 2) [one-shot]  (same as WiFi mode)
    TaskFFTProcessor (pri 1) [one-shot]
    TaskAdaptiveSampler (pri 2) [after FFT]
    TaskAggregator (pri 1)
    */
}

void loop() {
    vTaskDelete(NULL);
}
