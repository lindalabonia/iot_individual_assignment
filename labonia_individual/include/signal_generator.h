#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include <Arduino.h>

// Write a 12-bit value to the MCP4725 via I2C fast-write
void mcp4725_write(uint16_t value);

// Build the lookup table of precomputed DAC values (called once in setup).
// Computes update rate and LUT size automatically from the signal frequencies.
void buildSignalLUT();

// FreeRTOS task: continuously outputs the signal through MCP4725
void TaskDACGenerator(void *pvParameters);

#endif
