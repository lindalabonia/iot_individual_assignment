#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "config.h"

Adafruit_INA219 ina219(INA219_ADDR);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);

    if (!ina219.begin(&Wire)) {
        Serial.println("INA219 not found — check wiring and address");
        while (true) delay(1000);
    }

    // 16V / 400mA range is enough for a 3.7V LiPo powering an ESP32
    ina219.setCalibration_16V_400mA();

    //Serial.println("ts_ms,bus_V,shunt_mV,current_mA,power_mW");
}

void loop() {
    // Variabile statica per memorizzare l'ultimo momento di risveglio
    static TickType_t xLastWakeTime = xTaskGetTickCount();
    // Conversione dei millisecondi definiti nel config in Ticks di sistema
    const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_INTERVAL_MS);

    // Attesa precisa: il task si sospende fino allo scoccare del prossimo intervallo "assoluto"
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    float mA = ina219.getCurrent_mA();

    Serial.printf(">mA: %.2f\r\n", mA); 
}