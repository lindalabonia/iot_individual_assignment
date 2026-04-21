#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

void setupWiFi();
void setupMQTT();
void mqttReconnect();
void mqttPublish(float avgV, int windowNum);

#endif
