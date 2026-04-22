#include "config.h"

#ifdef USE_LORAWAN

#include "lora_tx.h"
#include "globals.h"

#include <RadioLib.h>
#include <SPI.h>

// --- Radio hardware ---
// SX1262 is soldered on the Heltec V3 PCB (no external wiring).
// Dedicated SPI bus (HSPI) to avoid conflicts with other peripherals.
static SPIClass loraSPI(HSPI);
static SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// EU868 frequency plan (required for Europe / TTN)
static const LoRaWANBand_t* band = &EU868;
static LoRaWANNode node(&radio, band);

// OTAA credentials from config.h
static uint64_t joinEUI = LORA_JOIN_EUI;
static uint64_t devEUI  = LORA_DEV_EUI;
static uint8_t  appKey[] = LORA_APP_KEY;

// Called once from setup(). Initializes the SX1262 and joins TTN via OTAA.
void setupLoRa() {
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // Serial.print("[LORA] Initializing SX1262... ");
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        // Serial.printf("FAIL (%d)\n", state);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Heltec V3 board-specific setup:
    // - DIO2 controls the RF switch (antenna path)
    // - TCXO at 1.8V must be enabled, otherwise the oscillator frequency is off
    //   and the gateway can't decode the signal
    radio.setDio2AsRfSwitch(true);
    radio.setTCXO(1.8);

    node.setDatarate(LORA_DATARATE);

    // OTAA join: device sends a JoinRequest, waits for JoinAccept from TTN.
    // appKey passed twice: LoRaWAN 1.0.x uses one key for both roles (see config.h).
    node.beginOTAA(joinEUI, devEUI, appKey, appKey);

    // Retry join up to 5 times — first attempt can fail due to timing or gateway load
    // Serial.print("[LORA] Joining TTN (OTAA)... ");
    int retries = 5;
    while (retries > 0) {
        state = node.activateOTAA();
        if (state == RADIOLIB_LORAWAN_NEW_SESSION) break;
        // Serial.printf("FAIL (%d), retrying...\n", state);
        retries--;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (retries == 0) {
        // Serial.println("Could not join TTN. Check gateway, keys, and device registration.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Serial.println("OK");
}

// LoRa TX task — runs on Core 0, priority 1 (below DAC generator at pri 2).
// DAC preempts this task every ~5ms to write the next waveform sample, then
// yields via vTaskDelay — so signal generation is never interrupted, even
// during a multi-second LoRa transmission.
// In WiFi mode, the WiFi stack occupies Core 0 instead — they never coexist.
void TaskLoRaTX(void *pvParameters) {
    while (1) {
        TxFrame_t frame;
        // Block until aggregator posts a new result (every WINDOW_DURATION_SEC)
        xQueueReceive(txQueue, &frame, portMAX_DELAY);

        // --- Binary payload encoding (4 bytes instead of ~30 bytes JSON) ---
        // LoRaWAN max payload at SF12 is just 51 bytes, so compact encoding matters.
        //
        // float avgV (4 bytes) → multiply by 100, store as uint16 (2 bytes)
        //   e.g. 1.58V → 158. Resolution 0.01V, range 0-655V. Enough for 3.3V ADC.
        // int windowNum (4 bytes) → cast to uint16 (2 bytes)
        //   range 0-65535, enough for ~7.5 days of continuous 10s windows.
        //
        // TTN payload formatter decodes these bytes back to the original values.
        // Big-endian (MSB first) = network byte order, standard for LoRaWAN.
        uint16_t avgEncoded = (uint16_t)(frame.avgV * 100);
        uint16_t winEncoded = (uint16_t)(frame.windowNum);

        uint8_t payload[4];
        payload[0] = avgEncoded >> 8;
        payload[1] = avgEncoded & 0xFF;
        payload[2] = winEncoded >> 8;
        payload[3] = winEncoded & 0xFF;

        // sendReceive() is blocking: TX, then wait for two RX windows (Class A).
        // Standard LoRaWAN: RX1 at +1s, RX2 at +2s after TX.
        // TTN overrides this: RX1 at +5s, RX2 at +6s — the cloud network server
        // needs extra time to route the packet from gateway to server and back.
        // Total blocking time: ~6-7s. During this time, FreeRTOS scheduler still
        // runs the DAC generator (higher priority) via preemption — the signal
        // is not interrupted.
        // Radio occupation latency: how long sendReceive() blocks the device
        // (TX airtime + RX1 window at +5s + RX2 window at +6s on TTN).
        // Measured locally on the device, no gateway sync needed.
        uint32_t t0 = millis();
        int16_t state = node.sendReceive(payload, sizeof(payload), LORA_PORT);
        uint32_t delay_ms = millis() - t0;

        (void)state;
        (void)delay_ms;
        // if (state == RADIOLIB_ERR_NONE || state > 0) {
        //     Serial.printf("Window: %d  avg: %.3f V  delay: %lu ms\n",
        //                   frame.windowNum, frame.avgV, delay_ms);
        // } else {
        //     Serial.printf("Window: %d  FAILED state=%d\n",
        //                   frame.windowNum, state);
        // }

        // No extra delay: the aggregation window (10s) is the natural pacing.
        // sendReceive() takes ~7s, so LoRaTX finishes ~3s before the next
        // window average is ready — no values are missed.
    }
}

#endif // USE_LORAWAN
