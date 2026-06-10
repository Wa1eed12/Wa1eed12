#include "CAM_MT4.h"

// ============================================================
// CAM_MT4.ino — ESP32-CAM Waiter Robot Logger + Web Dashboard
// Board  : ESP32-CAM AI-Thinker (OV2640)
// Project: B — ESP32CAM_LOGGER
// Task   : MT-4
//
// FIRST-TIME SETUP — run this ONCE to provision credentials to NVS,
// then comment it out and reflash:
//
//   CONFIG_ProvisionCredentials(
//       "MyWiFiSSID", "MyWiFiPassword",
//       "admin",      "MyDashboardPassword"
//   );
//
// IMPORTANT: Disconnect UART wires (GPIO1/GPIO3) before flashing!
//            Reconnect after flash is complete.
// ============================================================

void setup() {
    // ── FIRST-TIME ONLY: uncomment to provision credentials, then re-comment ──
    // CONFIG_ProvisionCredentials("SSID", "PASS", "admin", "password");

    APP_CAM_Init();

    Serial.println("ESP32-CAM WAITER LOGGER — BOOT COMPLETE");
    if (WiFi.isConnected()) {
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.printf("Stream    : http://%s:%d/stream\n",
                      WiFi.localIP().toString().c_str(), WEB_STREAM_PORT);
        Serial.printf("Dashboard : http://%s:%d/\n",
                      WiFi.localIP().toString().c_str(), WEB_SERVER_PORT);
    } else {
        Serial.println("WiFi not connected — running in offline mode (SD logging only)");
    }
}

void loop() {
    APP_CAM_Run();
}
