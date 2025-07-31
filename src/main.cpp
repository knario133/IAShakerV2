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
    uiState = UI_MENU;
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print(WiFi.localIP());
  } else {
    lcd.print(language == 0 ? "No WiFi" : "No WiFi");
  }

  float cur, tgt;
  if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
    cur = currentRpm;
    tgt = targetRpm;
    xSemaphoreGive(rpmMutex);
  }
  char buf[17];
  snprintf(buf, sizeof(buf), "A:%3.0f T:%3.0f", cur, tgt);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(buf);
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
    if (idx >= menuCount) {
      lcd.print("                ");
    } else {
      lcd.print(idx == menuIndex ? ">" : " ");
      lcd.print(items[idx][language]);
      int len = strlen(items[idx][language]);
      for (int s = len; s < 15; ++s) lcd.print(' ');
    }
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

  lcd.setCursor(0, 0);
  lcd.print(language == 0 ? "Ajustar RPM" : "Adjust RPM");
  lcd.setCursor(0, 1);
  char buf[17];
  snprintf(buf, sizeof(buf), "RPM: %.0f", tempTarget);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(buf);

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

        if (millis() - lastCalc >= 500) {
            lastCalc = millis();
            long pos = stepper.getCurrentPositionInSteps();
            long deltaSteps = pos - lastPos;
            lastPos = pos;
            float rpm = (deltaSteps / (float)STEPS_PER_REV) * 120.0f;
            if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
                currentRpm = rpm;
                xSemaphoreGive(rpmMutex);
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
        StaticJsonDocument<200> doc;
        if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
            doc["currentRpm"] = currentRpm;
            doc["targetRpm"] = targetRpm;
            xSemaphoreGive(rpmMutex);
        }
        doc["wifi"] = (WiFi.status() == WL_CONNECTED);
        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
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

    WiFiManager wm;
    wifiConnected = wm.autoConnect("BioShaker_Config");
    if (!wifiConnected) {
        wm.startConfigPortal("BioShaker_Config");
        wifiConnected = (WiFi.status() == WL_CONNECTED);
    }

    setupServer();

    xTaskCreatePinnedToCore(uiTask, "uiTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motorTask, "motorTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // Nada que hacer aquí; FreeRTOS se encarga de las tareas
}

