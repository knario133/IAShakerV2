# BioShaker: Agitador de Laboratorio Inteligente

¡Bienvenido al repositorio del **BioShaker**! Este proyecto desarrolla un agitador de laboratorio automatizado y conectado, diseñado para ofrecer **precisión y flexibilidad** en la preparación de muestras biológicas, químicas o farmacéuticas. Puede controlarse tanto localmente mediante una pantalla **LCD y un encoder rotatorio**, como de forma remota a través de una **interfaz web (Wi-Fi)**. El sistema es robusto, fácil de usar y fiable, construido sobre la potencia del **ESP32 y FreeRTOS**.

---

## 🔬 Visión General del Proyecto

El BioShaker automatiza el proceso de agitación de líquidos, permitiendo un control preciso de la velocidad (RPM). Su doble interfaz de usuario garantiza que puede ser operado por cualquier persona en el laboratorio o monitoreado y ajustado desde cualquier lugar con conexión a la red.

---

## ✨ Características Principales

* **Control de RPM Preciso:** Agitación a velocidades controladas y estables.
* **Interfaz Local Intuitiva:** Configuración y monitoreo de RPM directamente en el dispositivo mediante LCD y encoder.
* **Conectividad Wi-Fi:** Acceso y control remoto vía navegador web.
* **Portal de Configuración:** Configuración de la red Wi-Fi sencilla a través de un portal cautivo.
* **Arquitectura Robusta:** Uso de **FreeRTOS** para un rendimiento multitarea fiable.
* **Multi-idioma:** Soporte para Español e Inglés en la interfaz local.
* **Versión del Firmware:** `v1.0.4`

---

## 🛠️ Hardware Necesario (Bill of Materials - BOM)

Para construir tu propio BioShaker, necesitarás los siguientes componentes:

* **Microcontrolador:** ESP32-WROOM-32 (o equivalente compatible).
* **Driver de Motor Paso a Paso:** DRV8825 (o A4988, TMC2208).
* **Motor Paso a Paso:** NEMA 17 (200 pasos/revolución, 1.8 grados/paso).
* **Módulo LCD I2C:** Pantalla LCD 16x2 con controlador PCF8574.
* **Encoder Rotatorio:** Con pulsador integrado.
* **Fuente de Alimentación:** 12V-24V DC, adecuada para el motor (ej., 12V 2A).
* **Regulador de Voltaje:** Si es necesario, para alimentar el ESP32 a 3.3V.
* **Cableado y Protoboard/PCB.**

### 📌 Conexión de Pines GPIO del ESP32

Asegúrate de conectar los componentes a los siguientes pines de tu ESP32:

* **Motor Paso a Paso (Driver DRV8825):**
    * `STEP_PIN`: GPIO 26
    * `DIR_PIN`: GPIO 27
    * `ENABLE_PIN`: GPIO 25 (Conectar a GND del driver para habilitarlo, o controlar con el ESP32 para activar/desactivar).
* **Módulo LCD I2C:**
    * `I2C_SDA`: GPIO 21
    * `I2C_SCL`: GPIO 22
* **Encoder Rotatorio:**
    * `ENC_CLK`: GPIO 5
    * `ENC_DT`: GPIO 18
    * `ENC_SW`: GPIO 19 (Configurado con `INPUT_PULLUP`).

---

## 💻 Configuración del Software y Entorno de Desarrollo

Este proyecto está diseñado para ser compilado con el **framework de Arduino para ESP32**.

### 1\. Requisitos Previos

* **Arduino IDE** (o PlatformIO).
* **Soporte de placas ESP32** instalado en tu IDE.

### 2\. Instalación de Librerías

Necesitarás instalar las siguientes librerías a través del **Gestor de Librerías de Arduino IDE** (Sketch \> Incluir Librería \> Gestionar Librerías) o `platformio.ini` si usas PlatformIO:

* `ESP_FlexyStepper` (por Joel G. Reuber)
* `U8g2` (por oliver) - **¡Importante\!** Este proyecto utiliza el constructor `U8G2_PCF8574_16X2_1_F_4W_HW_I2C` para la pantalla LCD. Asegúrate de que tu versión de la librería U8g2 sea compatible con este constructor o consulta los ejemplos de la librería para encontrar el adecuado para tu LCD/PCF8574.
* `ESP32Encoder` (por madhephaestus)
* `WiFiManager` (por Tzapu)
* `ESPAsyncWebServer` (por me-no-dev)
* `ArduinoJson` (por Benoit Blanchon)
* `Wire` (Ya viene incluido con Arduino IDE para comunicación I2C).

### 3\. Cargar el Firmware

1.  Abre el archivo `main.cpp` (o el `.ino`) del proyecto en tu IDE.
2.  Selecciona la placa **`ESP32 Dev Module`** (o tu placa ESP32 específica) en el menú de Herramientas.
3.  Selecciona el puerto serie correcto para tu ESP32.
4.  Sube el código a tu ESP32.

### 4\. Preparar el Sistema de Archivos (SPIFFS)

El servidor web necesita el archivo `index.html` (y potencialmente otros archivos estáticos) almacenado en el sistema de archivos SPIFFS del ESP32.

1.  Crea una carpeta llamada `data` en el directorio raíz de tu proyecto (donde está `main.cpp`).
2.  Coloca tu archivo `index.html` (y cualquier otro archivo estático para el servidor web) dentro de esta carpeta `data`.
3.  Instala el plugin **"ESP32 Sketch Data Upload"** para Arduino IDE:
    * Ve a la [página de releases](https://github.com/espressif/arduino-esp32fs-plugin/releases) del plugin.
    * Descarga la versión más reciente del archivo `.jar`.
    * Crea una carpeta `tools` dentro de tu directorio de *sketchbook* de Arduino (normalmente `Documents/Arduino`).
    * Dentro de `tools`, crea otra carpeta llamada `ESP32FS`.
    * Copia el archivo `.jar` descargado a la carpeta `ESP32FS`.
    * Reinicia el Arduino IDE.
4.  Ahora deberías ver la opción **`ESP32 Sketch Data Upload`** en el menú **Herramientas**.
5.  Haz clic en esta opción para subir la carpeta `data` a SPIFFS.

---

## 🚀 Uso del BioShaker

Una vez que el firmware está cargado y el SPIFFS está configurado:

### En el Dispositivo (LCD y Encoder):

1.  **Pantalla de Inicio:** Verás el logo y la versión del firmware.
2.  **Pantalla Normal:** Mostrará el estado de Wi-Fi, la dirección IP (si está conectado) y la RPM actual del motor.
3.  **Acceder al Menú:** Pulsa el **botón del encoder** para entrar al menú principal.
4.  **Navegar por el Menú:** Gira el encoder para desplazarte por las opciones.
5.  **Seleccionar Opción:** Pulsa el botón del encoder para confirmar tu selección.
6.  **Ajustar RPM:** Dentro de "Ajustar RPM", gira el encoder para cambiar el valor y pulsa el botón para confirmar.
7.  **Configurar Wi-Fi:** Dentro de "Configurar WiFi", pulsa el botón para activar el portal de configuración.

### Control Remoto (Vía Wi-Fi):

1.  **Conexión Inicial:**
    * Si es la primera vez o las credenciales guardadas son incorrectas, el BioShaker iniciará un punto de acceso Wi-Fi llamado **`BioShaker_Config`**.
    * Conéctate a esta red desde tu teléfono o computadora.
    * Se abrirá automáticamente (o deberás abrir un navegador y navegar a `192.168.4.1`) el portal de configuración donde podrás seleccionar tu red Wi-Fi e introducir la contraseña.
    * Una vez configurado, el ESP32 se reiniciará e intentará conectarse a tu red.
2.  **Acceso al Servidor Web:**
    * Una vez que el BioShaker esté conectado a tu red Wi-Fi, su dirección IP se mostrará en la pantalla LCD.
    * Abre un navegador web en cualquier dispositivo conectado a la misma red e introduce esa dirección IP (ej., `http://192.168.1.100`).
    * Verás la interfaz web (`index.html`) desde donde podrás monitorear y controlar la RPM.
3.  **Endpoints de la API:**
    * **`GET /rpm?value=<RPM>`:** Envía una solicitud GET para establecer la RPM del motor.
        * Ejemplo: `http://<IP_DEL_BIOSHAKER>/rpm?value=250`
    * **`GET /status`:** Obtiene el estado actual del agitador en formato JSON.
        * Respuesta de ejemplo: `{"currentRpm":125.5,"targetRpm":150.0,"wifi":true}`

---

## ⚙️ Arquitectura de Software

El firmware del BioShaker está construido sobre **FreeRTOS**, un sistema operativo en tiempo real que permite una gestión eficiente y concurrente de las diferentes funcionalidades:

* **`uiTask` (Núcleo 0):** Gestiona todas las interacciones de la interfaz de usuario, incluyendo el LCD y el encoder.
* **`motorTask` (Núcleo 1):** Controla el motor paso a paso, aplicando la RPM deseada y calculando la RPM actual.
* **Sincronización:** Se utiliza un **Mutex (`rpmMutex`)** para proteger el acceso a las variables compartidas como `targetRpm` y `actualRpm` entre las diferentes tareas (UI, Motor, Servidor Web) y evitar condiciones de carrera, garantizando la integridad de los datos.

