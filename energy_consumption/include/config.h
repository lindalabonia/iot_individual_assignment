#pragma once

// I2C pins (same physical header as main project)
#define PIN_SDA         41
#define PIN_SCL         42
#define I2C_FREQ_HZ     400000

// INA219 default address (A0=A1=GND)
#define INA219_ADDR     0x40

// How often to read and print a sample
// 100 = 10 Hz, 1000 = 1 Hz, 5000 = 0.2 Hz
#define SAMPLE_INTERVAL_MS  100 

// Shunt resistor on the INA219 module (R100 label = 0.1 Ω)
// The Adafruit library assumes 0.1 Ω by default — no override needed
// unless you use a different module.