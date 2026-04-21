#ifndef LORA_TX_H
#define LORA_TX_H

#include "config.h"

#ifdef USE_LORAWAN

void setupLoRa();
void TaskLoRaTX(void *pvParameters);

#endif
#endif
