// IoT Adaptive Sampling System
// Heltec WiFi LoRa 32 V3 + MCP4725 DAC (loopback: DAC out -> ADC in)


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
    Serial.println("\n=== IoT Adaptive Sampling System ===\n");


    
    // 1. TRANSMISSION INIT

#ifdef SKIP_TX
     Serial.println("[MODE] No transmission (SKIP_TX), serial output only");
#elif defined(USE_LORAWAN)
    setupLoRa();
#else
    // WiFi + MQTT — persistent connection, established once at boot.
    setupWiFi();
    setupMQTT();
#endif

    // 2. HARDWARE INIT (I2C, ADC, DAC)
    Wire.begin(I2C_SDA, I2C_SCL);

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

    Serial.printf("Signal: ");
    for (int k = 0; k < NUM_COMPONENTS; k++) {
        Serial.printf("%.1f*sin(2pi*%.1ft)", amplitudes[k], frequencies[k]);
        if (k < NUM_COMPONENTS - 1) Serial.printf(" + ");
     }
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
    // Adaptive mode : TaskOversampler and TaskFFTProcessor run one-shot on core 1
    // TaskAdaptiveSampler is created inside TaskFFTProcessor after f_max is known.
    
    xTaskCreatePinnedToCore(TaskOversampler,  "Sample",  8192, NULL, 2, NULL,            1);
  #ifdef BONUS
    // in BONUS mode the FFT task is not created - the FFT methods are called inside TaskFilter
    // Once the FFT is done and the f_max is found, TaskFilter creates the AdaptiveSampler task directly 
    // (inside applyAdaptiveRate, which is defined in fft_processor.cpp)
    xTaskCreatePinnedToCore(TaskFilter,       "Filter", 16384, NULL, 1, &filterTaskHandle, 1);
  #else
    xTaskCreatePinnedToCore(TaskFFTProcessor, "FFT",     8192, NULL, 1, &fftTaskHandle,  1);
  #endif

#else
    // Oversampling mode: TaskOversampler and TaskFFTProcessor are not created. 
    // TaskAdaptiveSampler is created now with the max sampling rate 
    // (adaptiveSamplingFreq defaults to MAX_SAMPLING_FREQ, set in globals.cpp).

    fftDone = true; // to unblock TaskAggregator, even if FFT is skipped
    xTaskCreatePinnedToCore(TaskAdaptiveSampler, "AdaptSamp", 4096, NULL, 2, &adaptiveTaskHandle, 1);
    // Serial.printf("[MODE] Oversampling at %.0f Hz (FFT skipped)\n", MAX_SAMPLING_FREQ);
#endif

    xTaskCreatePinnedToCore(TaskAggregator, "Agg", 4096, NULL, 1, NULL, 1);

#if defined(USE_LORAWAN) && !defined(SKIP_TX)
    // LoRa TX on Core 0, pri 1 (below DAC pri 2).
    xTaskCreatePinnedToCore(TaskLoRaTX, "LoRaTX", 8192, NULL, 1, NULL, 0);
#endif

}

void loop() {
    vTaskDelete(NULL);
}
