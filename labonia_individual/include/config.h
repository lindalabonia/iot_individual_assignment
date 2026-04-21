#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Transmission mode ---
//#define USE_LORAWAN    // LoRaWAN + TTN (if commented out → WiFi + MQTT)
#define SKIP_TX        // No transmission 

// --- Sampling mode ---
//#define SKIP_FFT       // Oversampling at MAX_SAMPLING_FREQ (if commented out → adaptive)

// --- Bonus: anomaly filtering ---
// When enabled, the oversampler buffer is contaminated with Gaussian noise
// and sparse spike anomalies BEFORE the FFT. Two filters (Z-score, Hampel)
// are evaluated on the same buffer; the one selected by ACTIVE_FILTER
// drives the adaptive pipeline downstream.
//#define BONUS

#ifdef BONUS
    // n(t) ~ N(0, sigma). 0.2 V → 248 LSB (0.2 / 3.3 * 4095)
    #define NOISE_SIGMA_LSB    248.0f

    // A(t): sparse spikes, sign random, magnitude ~ U(SPIKE_MIN, SPIKE_MAX),
    // injected with Bernoulli(p) per sample
    #define ANOMALY_PROB       0.1f     // test matrix: 0.01, 0.05, 0.10
    #define SPIKE_MIN_LSB      6200.0f   // 5 V in LSB
    #define SPIKE_MAX_LSB      18600.0f  // 15 V in LSB

    // Sliding window for both filters (centered on current sample).
    #define FILTER_WINDOW      40         // 5, 15, 40, 100
    
    // |x[i] - mean| / std > 3.0  →  anomaly
    #define ZSCORE_THRESHOLD   3.0f 
    
    // |x[i] - median| / (1.4826 × MAD) > 3.0  →  anomaly
    // MAD = median absolute deviation = median(|x[i] - median|)
    #define HAMPEL_THRESHOLD   3.0f      

    // Which filtered buffer feeds the adaptive sampler after the bonus phase
    #define FILTER_ZSCORE      0
    #define FILTER_HAMPEL      1
    #define ACTIVE_FILTER      FILTER_HAMPEL
#endif

// --- Pin definitions (Heltec WiFi LoRa 32 V3 - ESP32-S3) ---
#define I2C_SDA      41    // Heltec V3 I2C SDA
#define I2C_SCL      42    // Heltec V3 I2C SCL
#define ADC_PIN      2     // GPIO2 = ADC1_CH1, clean pin (GPIO1 has VBAT circuitry)
#define MCP4725_ADDR 0x60  // MCP4725 default I2C address

// --- Signal: s(t) = sum of a_k * sin(2*pi*f_k*t) ---
#define NUM_COMPONENTS 3
extern const float amplitudes[NUM_COMPONENTS];
extern const float frequencies[NUM_COMPONENTS];

// --- Sampling & FFT ---
#define FFT_SAMPLES        512    // Must be power of 2
#define MAX_SAMPLING_FREQ  500.0  // Hz, initial oversampling rate

// Fraction of the peak FFT magnitude used as threshold to separate real signal
// components from noise/leakage. A bin is considered significant only if its
// magnitude is at least this fraction of the highest bin's magnitude.
// e.g. 0.05 = 5%: if the strongest bin has mag=165000, the threshold becomes
// 8250 — only bins above 8250 are treated as real frequency components.
// A relative threshold adapts automatically to different signal amplitudes
// and ADC configurations, unlike a fixed value which would need manual tuning.
#define FFT_MAG_THRESHOLD_RATIO  0.05

// --- Aggregation ---
#define WINDOW_DURATION_SEC 30

// --- DAC output ---
#define DAC_OFFSET           2048  // Mid-scale = 1.65V
#define DAC_AMPLITUDE        1800  // Keep output within 0-4095
// Points per cycle of the highest frequency — controls waveform smoothness.
// 40 is empirical: enough for a smooth sine, low enough for I2C overhead.
// The actual update rate and LUT size are computed at runtime in buildSignalLUT()
// based on the signal frequencies, so changing the signal does not require
// changing any other constant here.
#define DAC_POINTS_PER_CYCLE 40
#define DAC_LUT_MAX          2048  // max LUT entries (compile-time array bound)

// --- WiFi + MQTT (only used when USE_LORAWAN is NOT defined) ---
#ifndef USE_LORAWAN
//#define WIFI_SSID     "REDACTED"
//#define WIFI_PASS     "REDACTED"
#define WIFI_SSID     "REDACTED"
#define WIFI_PASS     "REDACTED"
//#define MQTT_BROKER   "REDACTED"  // IP of PC running Mosquitto
#define MQTT_BROKER   "REDACTED"  
#define MQTT_PORT     1883
#define MQTT_TOPIC    "iot/sensor/avg"
#endif

// --- LoRaWAN + TTN (only used when USE_LORAWAN IS defined) ---
#ifdef USE_LORAWAN

// OTAA keys from TTN Console — MSB format.
// LoRaWAN 1.0.x uses a single key (AppKey) for both app and network encryption.
// RadioLib wants two key parameters; we pass the same AppKey for both.
#define LORA_JOIN_EUI  0x0000000000000000
#define LORA_DEV_EUI   REDACTED
#define LORA_APP_KEY   { REDACTED, \
                         REDACTED }

// LoRaWAN uses "data rate" (DR) instead of SF directly, because DR encodes
// both spreading factor and bandwidth in a single index.
// EU868: DR5=SF7/125kHz, DR4=SF8, ..., DR0=SF12.
// SF7 chosen: shortest air time (~50ms for 4 bytes), enough range for same building.
#define LORA_DATARATE  5
#define LORA_PORT      1     // LoRaWAN FPort (1-223)

// TX pacing: no artificial delay between transmissions. We send once per
// aggregation window (WINDOW_DURATION_SEC). The math for a test session:
//   ToA at SF7 with 4-byte payload ≈ 50ms
//   Duty cycle 1%: max 36s/hour airtime → ~720 pkt/hour → OK at 1 pkt/10s
//   TTN fair use: 30s airtime/day → 600 packets → ~100 min of continuous test

// Heltec V3 SX1262 pins (on-board, no external wiring needed)
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10

#endif

#endif
