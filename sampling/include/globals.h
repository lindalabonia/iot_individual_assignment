#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// --- Shared state ---
// These are "volatile" because they are written by one task and read by others
// running on different cores. Without volatile, the compiler might cache the
// value in a register and a task would never see the update from another task.
extern volatile float adaptiveSamplingFreq;
extern volatile float detectedMaxFreq;
extern volatile bool  fftDone;

// --- FFT buffers (filled by sampler, consumed by FFT task) ---
extern double vReal[];
extern double vImag[];

// --- Aggregated result (passed from aggregator to LoRa TX task) ---
typedef struct {
    float avgV;
    int   windowNum;
} TxFrame_t;

// --- Synchronization ---
// sampleQueue: FIFO of float values, written by TaskAdaptiveSampler (Core 1),
// read by TaskAggregator (Core 1). Each element is one raw ADC reading.
extern QueueHandle_t sampleQueue;

// txQueue: single-slot queue of TxFrame_t, written by TaskAggregator,
// read by TaskLoRaTX. Size=1 with xQueueOverwrite: aggregator never blocks,
// LoRaTX always gets the most recent average (older ones are discarded).
// Only used in LoRaWAN mode — MQTT calls mqttPublish() directly because
// the publish takes ~1-5ms and doesn't need a separate task.
// LoRa TX takes seconds (air time + RX windows + duty cycle wait),
// so it must run in its own task to not block the aggregator.
#ifdef USE_LORAWAN
extern QueueHandle_t txQueue;
#endif
// fftTaskHandle: used by TaskOversampler to notify TaskFFTProcessor via
// xTaskNotifyGive() that the FFT buffer is ready to be analysed.
extern TaskHandle_t  fftTaskHandle;
// adaptiveTaskHandle: set when TaskFFTProcessor creates TaskAdaptiveSampler
// dynamically. Kept for potential future use (e.g. deleting the task).
extern TaskHandle_t  adaptiveTaskHandle;

// filterTaskHandle: notified by TaskOversampler when BONUS is active.
// The filter task replaces the FFT task in the boot pipeline: it injects
// noise/anomalies, evaluates Z-score and Hampel, then runs FFT itself.
#ifdef BONUS
extern TaskHandle_t  filterTaskHandle;
#endif

#endif
