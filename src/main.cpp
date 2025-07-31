#include <Arduino.h>
#include <ESP_FlexyStepper.h>
#include <U8g2lib.h> // Librería U8g2
#include <ESP32Encoder.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Wire.h>

// Versión del firmware
#define FIRMWARE_VERSION "1.0.4" // Actualízalo según sea necesario para tu proyecto

// Definiciones de pines
#define DIR_PIN 27
#define STEP_PIN 26
#define ENABLE_PIN 25 // Generalmente activo en LOW para habilitar el driver

#define I2C_SDA 21
#define I2C_SCL 22
// Original constructor from your first paste. This is a common and usually correct one.
// The address (0x3F or 0x27) is handled internally by the U8g2_PCF8574 constructor.
// U8X8_PIN_NONE means no reset pin is used, which is typical for I2C LCDs.
U8G2_PCF8574_16X2_1_F_4W_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // <-- Reverted to this

#define ENC_CLK 5
#define ENC_DT 18
#define ENC_SW 19

// Configuración del motor paso a paso
const int STEPS_PER_REV = 6200; // Motor de 200 pasos/rev con microstepping 1/16 = 3200 pasos/rev (ajustado para 6200 pasos)
const float MAX_RPM = 510.0;
const float ACCEL = 800.0; // pasos/s^2

// Variables globales
ESP_FlexyStepper stepper;

ESP32Encoder encoder;
WiFiManager wm;
AsyncWebServer server(80);

struct WifiCredentials {
  String ssid;
  String pass;
};

bool configMode = false;

// Prototipos de funciones
void handleSplash();
void handleNormal();
void handleMenu();
void handleAdjustRpm();
void handleWifi();
void handleLanguage();
void uiTask(void *);
void motorTask(void *);
void stopMotor();
void pressDebounce(); // Ayuda para el rebote del botón

bool loadWifiConfig(WifiCredentials &cfg) {
  if (!SPIFFS.exists("/wifi.json")) return false;
  File f = SPIFFS.open("/wifi.json", "r");
  if (!f) return false;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  cfg.ssid = doc["ssid"].as<String>();
  cfg.pass = doc["password"].as<String>();
  return true;
}

void saveWifiConfig(const WifiCredentials &cfg) {
  File f = SPIFFS.open("/wifi.json", "w");
  if (!f) return;
  JsonDocument doc;
  doc["ssid"] = cfg.ssid;
  doc["password"] = cfg.pass;
  serializeJson(doc, f);
  f.close();
}

float targetRpm = 0.0f;
float tempTargetRpm = 0.0f;
float actualRpm = 0.0f;
SemaphoreHandle_t rpmMutex; // Mutex para las variables compartidas de RPM
bool uiNeedsRedraw = false; // Bandera para indicar que la UI necesita redibujarse

enum UiState { UI_SPLASH, UI_NORMAL, UI_MENU, UI_ADJUST_RPM, UI_WIFI, UI_LANGUAGE };
UiState uiState = UI_SPLASH; // Iniciar con la pantalla de presentación
int menuIndex = 0;
int language = 0; // 0: Español, 1: English

// Los textos del menú se usarán para la función userInterfaceSelectionList
// Asegúrate de que los arrays sean del tamaño correcto si agregas más elementos.
const char *menuItems_es[] = {"Ajustar RPM", "Detener Motor", "Configurar WiFi", "Idioma", ""}; // Último elemento vacío para userInterfaceSelectionList
const char *menuItems_en[] = {"Set RPM", "Stop Motor", "WiFi Setup", "Language", ""}; // Último elemento vacío

void setup() {
  Serial.begin(115200);

  // Configuración del stepper
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Driver siempre habilitado al inicio
  delay(10); // Darle tiempo al driver
  stepper.connectToPins(STEP_PIN, DIR_PIN);
  stepper.setStepsPerRevolution(STEPS_PER_REV);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCEL);
  stepper.setDecelerationInStepsPerSecondPerSecond(ACCEL);
  stepper.startAsService(1); // ejecutar en el núcleo 1
  Serial.println("Stepper initialized."); // Depuración

  // LCD y encoder
  Wire.begin(I2C_SDA, I2C_SCL); // Inicializar bus I2C para U8g2
  u8g2.begin(); // Inicializa la pantalla
  u8g2.enableUTF8Print(); // Habilitar soporte para caracteres como 'ñ'
  u8g2.setContrast(50); // Ajusta el contraste si es necesario (0-255)
  Serial.println("LCD initialized."); // Depuración

  encoder.attachHalfQuad(ENC_DT, ENC_CLK); // Half-quad para encoders simples
  encoder.clearCount();
  pinMode(ENC_SW, INPUT_PULLUP); // Botón del encoder con pull-up interno
  Serial.println("Encoder initialized."); // Depuración

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed! Formatting...");
    SPIFFS.format(); // Intenta formatear si falla el montaje (borra todos los archivos)
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS re-mount failed! Restarting...");
      ESP.restart();
    }
  }
  Serial.println("SPIFFS initialized."); // Depuración

  rpmMutex = xSemaphoreCreateMutex(); // Crear mutex para variables compartidas
  if (rpmMutex == NULL) {
    Serial.println("Failed to create RPM mutex!"); // Depuración
  }

  // WiFi
  WifiCredentials cfg;
  if (loadWifiConfig(cfg)) {
    Serial.printf("Connecting to %s...\n", cfg.ssid.c_str());
    WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    configMode = WiFi.status() != WL_CONNECTED;
  } else {
    configMode = true;
  }

  if (configMode) {
    Serial.println("Starting config portal...");
    wm.setConfigPortalTimeout(180);
    if (wm.startConfigPortal("BioShaker_Config")) {
      WifiCredentials ncfg{WiFi.SSID(), WiFi.psk()};
      saveWifiConfig(ncfg);
      ESP.restart();
    } else {
      Serial.println("Config portal failed, restarting...");
      ESP.restart();
    }
  } else {
    Serial.print("WiFi Connected! IP: ");
    Serial.println(WiFi.localIP());

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server.on("/rpm", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (request->hasParam("value")) {
        float val = request->getParam("value")->value().toFloat();
        if (val < 0) val = 0;
        if (val > MAX_RPM) val = MAX_RPM;
        xSemaphoreTake(rpmMutex, portMAX_DELAY);
        targetRpm = val;
        xSemaphoreGive(rpmMutex);
      }
      request->send(200, "text/plain", "OK");
    });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
      JsonDocument doc;
      xSemaphoreTake(rpmMutex, portMAX_DELAY);
      doc["currentRpm"] = actualRpm;
      doc["targetRpm"] = targetRpm;
      xSemaphoreGive(rpmMutex);
      doc["wifi"] = true;
      String body;
      serializeJson(doc, body);
      request->send(200, "application/json", body);
    });

    server.begin();
    Serial.println("Web server started.");
  }

  // Iniciar tareas
  xTaskCreatePinnedToCore(uiTask, "UI", 4096, nullptr, 1, nullptr, 0); // UI en núcleo 0
  xTaskCreatePinnedToCore(motorTask, "Motor", 4096, nullptr, 1, nullptr, 1); // Motor en núcleo 1
  Serial.println("UI task started on core 0."); // Depuración
  Serial.println("Motor task started on core 1."); // Depuración
}

void loop() {
  // El bucle principal queda vacío, toda la lógica se ejecuta en tareas
  vTaskDelay(1);
}

// Tarea dedicada al control del motor - Núcleo 1
void motorTask(void *) {
  static uint32_t lastCalc = 0;
  static long lastPos = 0;
  static float appliedRpm = 0.0f; // Última RPM aplicada
  static bool jogging = false;     // Indica si se está en modo jogging

  for (;;) {
    float localTarget;
    xSemaphoreTake(rpmMutex, portMAX_DELAY); // Obtener la RPM objetivo actual de forma segura
    localTarget = targetRpm;
    xSemaphoreGive(rpmMutex);

    // Actualizar la velocidad del stepper solo si la RPM objetivo cambió
    // O si el estado de jogging necesita ser ajustado (por ejemplo, si se detuvo el motor)
    if (localTarget != appliedRpm || (localTarget == 0 && jogging)) {
      float stepsPerSec = (localTarget / 60.0f) * STEPS_PER_REV;

      if (localTarget > 0.0f) {
        stepper.setSpeedInStepsPerSecond(stepsPerSec); // Ajustar la velocidad
        if (!jogging) {
          stepper.startJogging(1); // Iniciar movimiento continuo
          jogging = true;
        }
      } else {
        stepper.stopJogging(); // Detener el movimiento
        jogging = false;
        stepper.setSpeedInStepsPerSecond(0.0f); // Asegurar que la velocidad se establece a 0
      }

      Serial.printf("Target changed, %.1f RPM -> %.1f RPM\n", appliedRpm, localTarget); // Depuración
      appliedRpm = localTarget;
    }

    // Calcular la RPM actual periódicamente
    if (millis() - lastCalc >= 500) { // Actualizar cada 500ms para lecturas más estables
      lastCalc = millis();
      long pos = stepper.getCurrentPositionInSteps();
      long delta = pos - lastPos; // Pasos dados en 500ms
      lastPos = pos;
      // Calcular RPM: (pasos/0.5s) / STEPS_PER_REV * 60s/min = (pasos / STEPS_PER_REV) * 120
      float rpm = (delta / (float)STEPS_PER_REV) * 120.0f; // Fórmula ajustada para 500ms
      xSemaphoreTake(rpmMutex, portMAX_DELAY);
      actualRpm = rpm;
      xSemaphoreGive(rpmMutex);
      // Serial.printf("Actual RPM: %.2f\n", actualRpm); // Descomentar para depuración más detallada
    }

    vTaskDelay(1); // Ceder tiempo al resto de tareas
  }
}

// Tarea de UI - Se ejecuta en el Núcleo 0
void uiTask(void *) {
  Serial.println("UI Task: Initial clear LCD."); // Depuración
  for (;;) {
    // Manejar la UI según el estado actual
    switch (uiState) {
      case UI_SPLASH:
        handleSplash();
        break;
      case UI_NORMAL:
        handleNormal();
        break;
      case UI_MENU:
        handleMenu();
        break;
      case UI_ADJUST_RPM:
        handleAdjustRpm();
        break;
      case UI_WIFI:
        handleWifi();
        break;
      case UI_LANGUAGE:
        handleLanguage();
        break;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Actualizar la UI aproximadamente cada 50 ms
  }
}

// Nuevo manejador para la pantalla de bienvenida
void handleSplash() {
  static uint32_t splashStartTime = 0;
  static bool splashDisplayed = false;

  if (!splashDisplayed) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB10_tr); // Usar una fuente más bonita
    u8g2.drawStr(0, 12, "BIOSAKER");
    char version_buf[10];
    snprintf(version_buf, sizeof(version_buf), "v%s", FIRMWARE_VERSION);
    u8g2.drawStr(u8g2.getDisplayWidth() - u8g2.getStrWidth(version_buf), 12, version_buf); // Alinear a la derecha
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 28, "Iniciando...");
    u8g2.sendBuffer();
    splashStartTime = millis();
    splashDisplayed = true;
    Serial.println("UI Task: Splash screen displayed."); // Depuración
  }

  if (millis() - splashStartTime >= 5000) { // Mostrar durante 5 segundos
    uiState = UI_NORMAL; // Transición a la UI normal
    uiNeedsRedraw = true; // Forzar redibujado para la UI normal
    splashDisplayed = false; // Reiniciar para el próximo arranque
    Serial.println("UI Task: Splash screen timed out. Transitioning to Normal UI."); // Depuración
  }
  // No se procesa entrada del encoder durante la pantalla inicial
}

void handleNormal() {
  // Limpiar el buffer y dibujar en cada ciclo para asegurar que la pantalla se actualice.
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Mostrar estado WiFi
  bool wifi = WiFi.status() == WL_CONNECTED;
  u8g2.drawStr(0, 10, "WiFi:");
  u8g2.drawStr(35, 10, wifi ? (language == 0 ? "Conectado" : "Connected") : (language == 0 ? "Sin Conexión" : "No Connection"));

  if (wifi) {
    String ip = WiFi.localIP().toString();
    u8g2.drawStr(0, 26, ip.c_str());
  } else {
    String msg = language == 0 ? "Sin WiFi" : "No WiFi";
    u8g2.drawStr(0, 26, msg.c_str());
  }

  // Mostrar RPM actual
  char rpm_buf[16];
  xSemaphoreTake(rpmMutex, portMAX_DELAY);
  snprintf(rpm_buf, sizeof(rpm_buf), "RPM: %.1f", actualRpm);
  xSemaphoreGive(rpmMutex);
  u8g2.drawStr(u8g2.getDisplayWidth() - u8g2.getStrWidth(rpm_buf), 10, rpm_buf); // Ajustar posición si es necesario para que no se superponga

  u8g2.sendBuffer(); // Enviar todo el buffer a la pantalla

  if (digitalRead(ENC_SW) == LOW) {
    pressDebounce();
    menuIndex = 0; // Resetear el índice del menú al entrar
    uiState = UI_MENU;
    uiNeedsRedraw = true; // Forzar redibujado para la pantalla de menú
    Serial.println("UI Task: Entered Menu from Normal screen."); // Depuración
  }
}

void handleMenu() {
  // Obtener el estado del encoder y el botón
  int32_t currentEncoderCount = encoder.getCount();
  bool buttonPressed = (digitalRead(ENC_SW) == LOW);

  const char **currentMenuItems = (language == 0) ? menuItems_es : menuItems_en;
  const char *menuTitle = (language == 0) ? "Menú Principal" : "Main Menu";

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Usar userInterfaceSelectionList para manejar la navegación del menú
  // Se le pasa una función que devuelve el estado del encoder y el botón
  // y la lista de opciones. La función dibuja el menú y espera una selección.
  uint8_t selected_item = u8g2.userInterfaceSelectionList(
    menuTitle,
    1, // Elemento preseleccionado (1-based index)
    currentMenuItems
  );

  u8g2.sendBuffer(); // Asegurarse de que el menú se dibuje

  // Actuar si se ha seleccionado un elemento
  if (selected_item) {
    menuIndex = selected_item - 1; // La función devuelve base 1, lo convertimos a base 0
    switch (menuIndex) {
      case 0:
        uiState = UI_ADJUST_RPM;
        break;
      case 1:
        stopMotor();
        uiState = UI_NORMAL;
        break;
      case 2:
        uiState = UI_WIFI;
        break;
      case 3:
        uiState = UI_LANGUAGE;
        break;
    }
    Serial.printf("UI Task: Menu item %d selected, new state: %d.\n", menuIndex, uiState); // Depuración
    uiNeedsRedraw = true; // Forzar redibujado para el siguiente estado
  } else {
    // Si no se selecciona nada y se pulsa atrás (o el encoder no se mueve y se pulsa atrás),
    // esto no está implementado directamente con userInterfaceSelectionList.
    // Si necesitas un "atrás", podrías añadir una opción de menú "Volver" o
    // usar un botón dedicado fuera del control de U8g2 para cambiar el estado.
  }
}

void handleAdjustRpm() {
  static long lastEncoderCount = encoder.getCount(); // Almacenar el conteo del encoder la última vez que se entró
  static bool entered = true; // Para inicializar tempTargetRpm solo una vez al entrar

  if (entered || uiNeedsRedraw) {
    xSemaphoreTake(rpmMutex, portMAX_DELAY);
    tempTargetRpm = targetRpm; // Inicializar con el valor actual del motor
    xSemaphoreGive(rpmMutex);
    encoder.clearCount(); // Limpiar el encoder al entrar
    lastEncoderCount = 0; // Resetear la referencia para el cálculo del delta
    uiNeedsRedraw = false;
    entered = false;
    Serial.printf("UI Task: Adjust RPM UI entered/redrawn, initial tempTargetRpm: %.1f.\n", tempTargetRpm); // Depuración
  }

  // Leer el cambio del encoder desde el último ciclo
  int32_t currentEncoderCount = encoder.getCount();
  int32_t delta = currentEncoderCount - lastEncoderCount;
  lastEncoderCount = currentEncoderCount; // Actualizar el último conteo

  if (delta != 0) {
    tempTargetRpm += (float)delta; // Ajustar RPM por los pasos del encoder
    if (tempTargetRpm < 0) tempTargetRpm = 0; // Limitar RPM mínima
    if (tempTargetRpm > MAX_RPM) tempTargetRpm = MAX_RPM; // Limitar RPM máxima
    // Serial.printf("UI Task: Encoder adjusted tempTargetRpm to %.1f.\n", tempTargetRpm); // Depuración
  }

  // Dibujar con U8g2
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB10_tr); // Título más grande
  u8g2.drawStr(0, 12, language == 0 ? "Ajustar RPM" : "Set RPM");

  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f RPM", tempTargetRpm);
  u8g2.setFont(u8g2_font_7x13_tf); // Fuente para los números
  u8g2.drawStr(0, 28, buf);

  u8g2.sendBuffer();

  if (digitalRead(ENC_SW) == LOW) { // Botón presionado
    pressDebounce(); // Antirrebote
    xSemaphoreTake(rpmMutex, portMAX_DELAY); // Actualizar la RPM objetivo global
    targetRpm = tempTargetRpm;
    xSemaphoreGive(rpmMutex);
    uiState = UI_NORMAL; // Volver a la pantalla normal
    uiNeedsRedraw = true; // Forzar redibujado para el siguiente estado
    entered = true; // Restablecer la bandera para la próxima vez que se entre
    Serial.printf("UI Task: RPM adjusted to %.1f and saved. Returning to Normal UI.\n", targetRpm); // Depuración
  }
}

void handleWifi() {
  // Verifica si el estado de la UI ha cambiado para redibujar completamente
  static bool entered = true; // Para asegurar que el mensaje inicial se dibuje una vez
  if (entered || uiNeedsRedraw) {
    uiNeedsRedraw = false;
    entered = false;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, language == 0 ? "Configurando WiFi" : "WiFi Config");
    u8g2.drawStr(0, 26, "AP: BioShaker_Conf");
    u8g2.sendBuffer();
    Serial.println("UI Task: WiFi Config UI redraw triggered (state change or flag)."); // Depuración
  }

  Serial.println("UI Task: Prompting for WiFi Config Portal..."); // Depuración

  if (digitalRead(ENC_SW) == LOW) { // Botón presionado
    pressDebounce(); // Antirrebote
    Serial.println("UI Task: Starting WiFi Config Portal..."); // Depuración
    wm.setConfigPortalTimeout(180); // Asegura un tiempo de espera consistente
    wm.startConfigPortal("BioShaker_Config");
    // Después del portal de configuración, verifica el estado de conexión
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi Config Portal Exited. Connected to: ");
      Serial.println(WiFi.SSID());
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WiFi Config Portal Exited. Not connected.");
    }
    uiState = UI_NORMAL; // Volver a la interfaz normal tras la configuración
    uiNeedsRedraw = true; // Forzar redibujado para la interfaz normal
    entered = true; // Restablecer la bandera
  }
}

void handleLanguage() {
  static bool entered = true;
  if (entered || uiNeedsRedraw) {
    uiNeedsRedraw = false;
    entered = false;
    // Limpiar el contador del encoder al entrar para evitar cambios inesperados
    encoder.clearCount();
    Serial.println("UI Task: Language UI redraw triggered (state change or flag)."); // Depuración
  }

  int32_t delta = encoder.getCount();
  if (delta != 0) {
    encoder.clearCount();
    language = !language; // Alternar idioma (0 o 1)
    Serial.printf("UI Task: Language changed to %s.\n", language == 0 ? "Español" : "English"); // Depuración
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Idioma/Language");
  u8g2.drawStr(0, 26, language == 0 ? "> Español" : "  Español");
  u8g2.drawStr(60, 26, language == 1 ? "> English" : "  English");
  u8g2.sendBuffer();

  if (digitalRead(ENC_SW) == LOW) { // Botón presionado
    pressDebounce(); // Antirrebote
    uiState = UI_NORMAL; // Volver a la pantalla normal
    uiNeedsRedraw = true; // Forzar redibujado para el siguiente estado
    entered = true; // Restablecer la bandera
    Serial.println("UI Task: Language selection confirmed. Returning to Normal UI."); // Depuración
  }
}

void stopMotor() {
  xSemaphoreTake(rpmMutex, portMAX_DELAY);
  targetRpm = 0; // Establecer RPM objetivo a 0
  xSemaphoreGive(rpmMutex);
  Serial.println("stopMotor() called. targetRpm set to 0."); // Depuración
}

void pressDebounce() {
  unsigned long debounceTime = millis();
  while (digitalRead(ENC_SW) == LOW) {
    if (millis() - debounceTime > 5000) { // Ejemplo de detección de pulsación larga
      // Aquí podrías agregar lógica para una pulsación larga, como restablecer la configuración de fábrica.
      break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Esperar a que se suelte el botón
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); // Pequeña pausa tras soltar para evitar rebotes adicionales
}