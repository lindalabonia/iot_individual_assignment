#include "config.h"

#ifndef USE_LORAWAN

#include "wifi_mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

/*
// to measure latency
static volatile int lastAckWindow = -1;
// to measure latency: broker echoes our own publish back (we are subscribed
// to the same topic we publish on), extract window number from JSON payload.
static void onEcho(char* topic, byte* payload, unsigned int length) {
    char buf[64];
    unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);
    buf[n] = '\0';
    char* p = strstr(buf, "\"w\":");
    if (p) lastAckWindow = atoi(p + 4);
}
*/

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); // wait for the connection to establish
        // Serial.print(".");
    }
    // Serial.printf("\n[WIFI] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void setupMQTT() {
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    // Keepalive must be LONGER than the publish interval, otherwise the broker
    // drops the connection between publishes. Default PubSubClient is 15 s.
    mqtt.setKeepAlive(2*WINDOW_DURATION_SEC);
    /*
    mqtt.setCallback(onEcho); // to measure latency
    */
    mqttReconnect();
}

// Limited-retry reconnect. A blocking while(!connected) would stall the
// aggregator forever if the broker is unreachable — the sample queue would
// fill up and the whole pipeline would freeze. With 5 attempts * 2 s = ~10 s
// max, one failed window is skipped at worst and the next window retries.
void mqttReconnect() {
    int attempts = 0;
    while (!mqtt.connected() && attempts < 5) {
        // Serial.print("[MQTT] Connecting...");
        if (mqtt.connect("ESP32_IoT")) {
            // Serial.println(" OK");
            /*
            mqtt.subscribe(MQTT_TOPIC); // to measure latency: self-subscribe to receive own publishes
            */
            return;
        }
        // Serial.printf(" FAIL (rc=%d)\n", mqtt.state());
        attempts++;
        delay(2000); // wait before retrying connection
    }
    // if (!mqtt.connected())
    //     Serial.println("[MQTT] Giving up this round, will retry next window");
}

// Publish window average as compact JSON.
void mqttPublish(float avgV, int windowNum) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();  // handle keepalive

    char payload[64];
    snprintf(payload, sizeof(payload),
             "{\"w\":%d,\"avg\":%.4f}", windowNum, avgV);

    // publish() returns false on failure (not connected, packet too large, TCP
    // error). Always check the return value — silently ignoring it would make
    // "Published" logs lie when the broker drops the connection mid-session.
    /*
    lastAckWindow = -1;                 // to measure latency
    unsigned long t0 = millis();        // to measure latency
    */
    bool ok = mqtt.publish(MQTT_TOPIC, payload);
    (void)ok;
    // Serial.printf("[MQTT] %s: %s\n", ok ? "Published" : "FAILED", payload);

    /*
    // to measure latency: wait for echo on iot/sensor/ack (max 500 ms)
    while (lastAckWindow != windowNum && (millis() - t0) < 500) {
        mqtt.loop();
        delay(1);
    }
    unsigned long delay_ms = millis() - t0;
    if (lastAckWindow == windowNum) {
        Serial.printf("Window: %d  avg: %.3f V  delay: %lu ms\n", windowNum, avgV, delay_ms);
    } else {
        Serial.printf("Window: %d  avg: %.3f V  delay: TIMEOUT\n", windowNum, avgV);
    }
    */
}

#endif // USE_LORAWAN
