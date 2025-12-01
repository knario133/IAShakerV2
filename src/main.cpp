#include <Arduino.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESP_FlexyStepper.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Encoder.h>
#include <Wire.h>

// Firmware info
#define FIRMWARE_VERSION "1.0.4"

// Pin definitions
#define DIR_PIN 27
#define STEP_PIN 26
#define ENABLE_PIN 25

#define I2C_SDA 21
#define I2C_SCL 22

#define ENC_CLK 5
#define ENC_DT 18
#define ENC_SW 19

const int STEPS_PER_REV = 6200;
const float MAX_RPM = 510.0f;

ESP_FlexyStepper stepper;
ESP32Encoder encoder;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variables compartidas
float targetRpm = 0.0f;
float currentRpm = 0.0f;
SemaphoreHandle_t rpmMutex;
bool wifiConnected = false;

enum UiState { UI_SPLASH, UI_NORMAL, UI_MENU, UI_ADJUST_RPM, UI_WIFI, UI_LANGUAGE };
UiState uiState = UI_SPLASH;
int language = 0;          // 0: Español, 1: English
int menuIndex = 0;

AsyncWebServer server(80);

// ---------------------------------------------------------------------------
// Helper functions for UI input
// ---------------------------------------------------------------------------
bool buttonPressed() {
  static uint32_t last = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - last > 200) {
    last = millis();
    return true;
  }
  return false;
}

int32_t readEncoder() {
  int32_t c = encoder.getCount();
  if (c != 0) {
    encoder.clearCount();
  }
  return c;
}

// ---------------------------------------------------------------------------
// UI state handlers
// ---------------------------------------------------------------------------
void handleSplash() {
  static bool first = true;
  static uint32_t start = 0;
  if (first) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BIOSAKER");
    lcd.setCursor(0, 1);
    lcd.print("v" FIRMWARE_VERSION);
    start = millis();
    first = false;
  }
  if (millis() - start > 5000) {
    first = true;
    uiState = UI_NORMAL;
  }
}

void handleNormal() {
  if (buttonPressed()) {
    menuIndex = 0;
    encoder.clearCount(); // Clear any accumulated movement
    uiState = UI_MENU;
    return;
  }

  char lineBuf[17];

  // Line 0: IP or Status
  // Optimization: Do not create String object every loop.
  if (WiFi.status() == WL_CONNECTED) {
    // Print IP directly into buffer using %u.%u.%u.%u or just use the IPAddress object which supports print,
    // but for snprintf padding we need a string.
    // IPAddress implements toString() but we want to avoid heap allocation.
    IPAddress ip = WiFi.localIP();
    // Format IP with padding directly into a temporary buffer, then copy to lineBuf if needed,
    // or just format directly with width.
    // However, %-16s expects a string argument.
    char ipStr[17];
    snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    snprintf(lineBuf, sizeof(lineBuf), "%-16s", ipStr);
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "%-16s", "No WiFi");
  }
  lcd.setCursor(0, 0);
  lcd.print(lineBuf);

  float cur, tgt;
  if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
    cur = currentRpm;
    tgt = targetRpm;
    xSemaphoreGive(rpmMutex);
  }

  // Line 1: RPMs
  char content[17];
  snprintf(content, sizeof(content), "A:%3.0f T:%3.0f", cur, tgt);
  snprintf(lineBuf, sizeof(lineBuf), "%-16s", content);
  lcd.setCursor(0, 1);
  lcd.print(lineBuf);
}

void handleMenu() {
  int32_t delta = readEncoder();
  menuIndex += delta;
  const int menuCount = 4;
  if (menuIndex < 0) menuIndex = menuCount - 1;
  if (menuIndex >= menuCount) menuIndex = 0;

  if (buttonPressed()) {
    switch (menuIndex) {
      case 0: uiState = UI_ADJUST_RPM; break;
      case 1:
        if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
          targetRpm = 0;
          xSemaphoreGive(rpmMutex);
        }
        uiState = UI_NORMAL;
        break;
      case 2: uiState = UI_WIFI; break;
      case 3: uiState = UI_LANGUAGE; break;
    }
    return;
  }

  const char *items[4][2] = {
    {"Ajustar RPM", "Adjust RPM"},
    {"Detener Motor", "Stop Motor"},
    {"Configurar WiFi", "WiFi Setup"},
    {"Idioma", "Language"}
  };

  int top = (menuIndex / 2) * 2;
  for (int i = 0; i < 2; ++i) {
    lcd.setCursor(0, i);
    int idx = top + i;
    char lineBuf[17];
    if (idx >= menuCount) {
      snprintf(lineBuf, sizeof(lineBuf), "%-16s", " ");
    } else {
      // Padded format: ">ItemText       " or " ItemText       "
      snprintf(lineBuf, sizeof(lineBuf), "%s%-15s", (idx == menuIndex ? ">" : " "), items[idx][language]);
    }
    lcd.print(lineBuf);
  }
}

void handleAdjustRpm() {
  static bool first = true;
  static float tempTarget = 0;
  if (first) {
    if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
      tempTarget = targetRpm;
      xSemaphoreGive(rpmMutex);
    }
    encoder.clearCount();
    lcd.clear();
    first = false;
  }

  int32_t delta = readEncoder();
  if (delta != 0) {
    tempTarget += delta;
    if (tempTarget < 0) tempTarget = 0;
    if (tempTarget > MAX_RPM) tempTarget = MAX_RPM;
  }

  // Optimize LCD update to prevent flicker
  lcd.setCursor(0, 0);
  char lineBuf[17];
  snprintf(lineBuf, sizeof(lineBuf), "%-16s", language == 0 ? "Ajustar RPM" : "Adjust RPM");
  lcd.print(lineBuf);

  lcd.setCursor(0, 1);
  char content[17];
  snprintf(content, sizeof(content), "RPM: %.0f", tempTarget);
  snprintf(lineBuf, sizeof(lineBuf), "%-16s", content);
  lcd.print(lineBuf);

  if (buttonPressed()) {
    if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
      targetRpm = tempTarget;
      xSemaphoreGive(rpmMutex);
    }
    first = true;
    uiState = UI_NORMAL;
  }
}

void handleWifi() {
  static bool first = true;
  if (first) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(language == 0 ? "Config WiFi" : "WiFi Setup");
    lcd.setCursor(0, 1);
    lcd.print("AP: BioShaker");
    first = false;
  }

  if (buttonPressed()) {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    wifiConnected = wm.startConfigPortal("BioShaker_Config");
    first = true;
    uiState = UI_NORMAL;
  }
}

void handleLanguage() {
  static bool first = true;
  static int sel = 0;
  if (first) {
    sel = language;
    encoder.clearCount();
    lcd.clear();
    first = false;
  }

  int32_t delta = readEncoder();
  if (delta != 0) {
    sel += delta;
    if (sel < 0) sel = 1;
    if (sel > 1) sel = 0;
  }

  lcd.setCursor(0, 0);
  lcd.print("Idioma/Language");
  lcd.setCursor(0, 1);
  if (sel == 0) {
    lcd.print(">Espa\xC3\xB1ol English");
  } else {
    lcd.print(" Espa\xC3\xB1ol >English");
  }

  if (buttonPressed()) {
    language = sel;
    first = true;
    uiState = UI_NORMAL;
  }
}

// ---------------------------------------------------------------------------
// Main UI task dispatch
// ---------------------------------------------------------------------------
void uiTask(void *parameter) {
  lcd.clear();
  while (true) {
    switch (uiState) {
      case UI_SPLASH:   handleSplash();   break;
      case UI_NORMAL:   handleNormal();   break;
      case UI_MENU:     handleMenu();     break;
      case UI_ADJUST_RPM: handleAdjustRpm(); break;
      case UI_WIFI:     handleWifi();     break;
      case UI_LANGUAGE: handleLanguage(); break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void motorTask(void *parameter) {
    static uint32_t lastCalc = 0;
    static long lastPos = 0;
    float applied = -1.0f;
    bool jogging = false;

    lastCalc = millis(); // Initialize to avoid huge delta on first run

    while (true) {
        float desired;
        if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
            desired = targetRpm;
            xSemaphoreGive(rpmMutex);
        }

        if (desired != applied) {
            float sps = (desired / 60.0f) * STEPS_PER_REV;
            if (desired > 0) {
                stepper.setSpeedInStepsPerSecond(sps);
                if (!jogging) {
                    stepper.startJogging(1);
                    jogging = true;
                }
            } else {
                stepper.stopJogging();
                jogging = false;
            }
            applied = desired;
        }

        uint32_t now = millis();
        uint32_t dt = now - lastCalc;
        if (dt >= 500) {
            long pos = stepper.getCurrentPositionInSteps();
            long deltaSteps = pos - lastPos;

            lastPos = pos;
            lastCalc = now;

            // Calculate RPM: (Delta Steps / Steps per Rev) * (60000ms / dt_ms)
            float rpm = 0.0f;
            if (dt > 0) {
                rpm = (deltaSteps / (float)STEPS_PER_REV) * (60000.0f / (float)dt);
            }

            // Protect shared variable update
            if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
                currentRpm = rpm;
                xSemaphoreGive(rpmMutex);
            }

            // Optional: Reconnect WiFi if lost (Non-blocking check)
            if (WiFi.status() != WL_CONNECTED && wifiConnected) {
                 // Logic to handle reconnection could go here, but WiFi.setAutoReconnect(true)
                 // (default on ESP32) handles transient loss.
                 // We rely on that for now to avoid blocking this task.
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setupServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/Endpoints", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/Endpoints.html", "text/html");
    });

    server.on("/rpm", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            float val = request->getParam("value")->value().toFloat();
            if (val < 0) val = 0;
            if (val > MAX_RPM) val = MAX_RPM;
            if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
                targetRpm = val;
                xSemaphoreGive(rpmMutex);
            }
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing value");
        }
    });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        StaticJsonDocument<300> doc;
        if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
            doc["currentRpm"] = currentRpm;
            doc["targetRpm"] = targetRpm;
            xSemaphoreGive(rpmMutex);
        }
        bool connected = (WiFi.status() == WL_CONNECTED);
        doc["wifi"] = connected;
        doc["ip"] = connected ? WiFi.localIP().toString() : "0.0.0.0";

        // Determine mode
        if (targetRpm > 0) {
            doc["mode"] = "Running";
        } else {
            doc["mode"] = "Idle";
        }

        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });

    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
        int n = WiFi.scanComplete();
        if(n == -2){
            // Not started, start async scan
            WiFi.scanNetworks(true);
            request->send(202, "application/json", "{\"status\":\"scanning\"}");
        } else if(n == -1){
            // Scanning
            request->send(202, "application/json", "{\"status\":\"scanning\"}");
        } else {
            // Done
            DynamicJsonDocument doc(1024);
            JsonArray array = doc.to<JsonArray>();
            for(int i=0; i<n; ++i){
                array.add(WiFi.SSID(i));
            }
            String json;
            serializeJson(doc, json);
            WiFi.scanDelete();
            request->send(200, "application/json", json);
        }
    });

    // Use POST for sensitive data
    server.on("/connect", HTTP_POST, [](AsyncWebServerRequest *request){
        String ssid, password;
        // Check params regardless of source (URL, Body, etc. as AsyncWebServer handles them uniformly if configured,
        // but for POST usually we check body or params. hasParam(..., true) checks body for POST.)
        if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
             ssid = request->getParam("ssid", true)->value();
             password = request->getParam("password", true)->value();
        } else if (request->hasParam("ssid") && request->hasParam("password")) {
             // Fallback to query params if not in body
             ssid = request->getParam("ssid")->value();
             password = request->getParam("password")->value();
        }

        if (ssid.length() > 0) {
            WiFi.begin(ssid.c_str(), password.c_str());
            request->send(200, "text/plain", "Connecting...");
        } else {
            request->send(400, "text/plain", "Missing params");
        }
    });

    // Archivos estáticos (CSS/JS/etc.)
    server.serveStatic("/", LittleFS, "/");

    server.begin();
}

void setup() {
    Serial.begin(115200);

    if (!LittleFS.begin()) {
        Serial.println("Failed to mount LittleFS");
    }

    // Hardware initialization
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.init();
    lcd.backlight();

    encoder.attachHalfQuad(ENC_DT, ENC_CLK);
    encoder.clearCount();
    pinMode(ENC_SW, INPUT_PULLUP);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    stepper.connectToPins(STEP_PIN, DIR_PIN);
    stepper.setStepsPerRevolution(STEPS_PER_REV);
    stepper.setAccelerationInStepsPerSecondPerSecond(800);
    stepper.setDecelerationInStepsPerSecondPerSecond(800);
    stepper.startAsService(1);

    rpmMutex = xSemaphoreCreateMutex();

    // Start tasks BEFORE WiFi to allow offline usage (LCD & Motor work immediately)
    xTaskCreatePinnedToCore(uiTask, "uiTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motorTask, "motorTask", 4096, NULL, 2, NULL, 1); // Priority 2 for Motor

    WiFiManager wm;
    wm.setConfigPortalTimeout(180); // 3 minutes timeout

    // Try to connect. If fails, open portal for timeout. Returns result.
    wifiConnected = wm.autoConnect("BioShaker_Config");

    if (wifiConnected) {
        setupServer();
    }
}

void loop() {
    // Nada que hacer aquí; FreeRTOS se encarga de las tareas
}

