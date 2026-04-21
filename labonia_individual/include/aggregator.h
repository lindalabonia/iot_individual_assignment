#ifndef AGGREGATOR_H
#define AGGREGATOR_H

#include <Arduino.h>

// Collects samples over a time window and computes average voltage.
void TaskAggregator(void *pvParameters);

#endif
