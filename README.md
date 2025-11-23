
# Sistema de Control de Paneo Térmico GSU con ESP32

Este proyecto implementa un sistema de control de movimiento para una cámara térmica montada sobre un servo motor SG90. El objetivo es inspeccionar dos unidades GSU (Generator Step-Up) en la base de un aerogenerador. El sistema soporta modos autónomo y manual controlados vía MQTT.

## 1. Especificaciones Geométricas y Montaje

Basado en las dimensiones proporcionadas (Profundidad: 2m, Ancho: 2.5m, Altura GSU: 3m), se define la siguiente disposición para optimizar la captura térmica:

*   **Ubicación de la Cámara:** Entrada del aerogenerador.
*   **Altura del Soporte:** **1.70 metros** del suelo (alineado al centro visual de los transformadores).
*   **Inclinación Vertical (Pitch):** **-5° a -8°** (leve inclinación hacia abajo).
*   **Ángulo de Paneo (Yaw):**
    *   **GSU Izquierda (ID 1):** El servo apunta a aprox. **125°**.
    *   **GSU Derecha (ID 2):** El servo apunta a aprox. **55°**.


## 2. Diagrama de Conexión (Hardware)

Conexión física entre la ESP32 y el servo motor SG90.

| Componente | Pin Servo (Color) | Pin ESP32 | Nota |
| :--- | :--- | :--- | :--- |
| **Señal PWM** | Naranja / Amarillo | **GPIO 18** | Definido en software |
| **VCC** | Rojo | **5V / VIN** | No usar 3.3V |
| **GND** | Marrón / Negro | **GND** | Tierra común |


## 3. Estructura del Proyecto (ESP-IDF)

Es crucial respetar esta jerarquía de carpetas para que el sistema de compilación `CMake` funcione correctamente.

```text
gsu_cam/                  <-- CARPETA RAÍZ (Ejecutar comandos aquí)
├── CMakeLists.txt        <-- Configuración del proyecto general
└── main/                 <-- Carpeta del componente principal
    ├── CMakeLists.txt    <-- Configuración del componente "main"
    └── main.c            <-- Código fuente en C
```


## 4. Archivos de Configuración y Código

Crea los archivos indicados en la estructura anterior con el siguiente contenido.

### A. `gsu_cam/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gsu_camera_control)
```

### B. `gsu_cam/main/CMakeLists.txt`
```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES mqtt nvs_flash esp_wifi driver)
```

### C. `gsu_cam/main/main.c`
```c
/*
 * GSU Thermal Camera Control - ESP32 ESP-IDF v5
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "mqtt_client.h"

// --- CONFIGURACIÓN DE RED ---
#define WIFI_SSID      "l17"
#define WIFI_PASS      "Kalev1501"
#define MQTT_BROKER_URI "mqtt://test.mosquitto.org" // CAMBIAR por tu broker

// --- TÓPICOS MQTT ---
#define TOPIC_MODE     "gsu/control/mode"    // Payload: "auto" | "manual"
#define TOPIC_MANUAL   "gsu/control/manual"  // Payload: "left" | "right"
#define TOPIC_STATUS   "gsu/data/status"     // Payload: "1" (Izq) | "2" (Der)

// --- CONFIGURACIÓN SERVO ---
#define SERVO_PIN      18
#define SERVO_FREQ     50
#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_RES       LEDC_TIMER_13_BIT

// --- PARÁMETROS DE MOVIMIENTO ---
#define ANGLE_LEFT_GSU  320  // ~125 grados
#define ANGLE_RIGHT_GSU 160  // ~55 grados
#define ANGLE_STEP      15   // Pasos manuales
#define SCAN_INTERVAL_MS 5000 // 5 segundos entre GSU

static const char *TAG = "GSU_CAM";
typedef enum { MODE_AUTO, MODE_MANUAL } system_mode_t;
system_mode_t current_mode = MODE_AUTO;
int current_pwm = ANGLE_LEFT_GSU;
bool just_switched_to_auto = true;

static esp_mqtt_client_handle_t mqtt_client;

// --- CONTROL DE SERVO ---
void servo_init() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RES, .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_PIN, .duty = 0, .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_servo_pwm(int duty) {
    if (duty < 100) duty = 100; 
    if (duty > 1000) duty = 1000;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    current_pwm = duty;
    ESP_LOGI(TAG, "Servo Pos: %d", duty);
}

// --- WIFI & MQTT EVENTOS ---
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconectando WiFi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi Conectado");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT Conectado");
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_MODE, 0);
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_MANUAL, 0);
    } else if (event_id == MQTT_EVENT_DATA) {
        char topic[64]; char data[64];
        sprintf(topic, "%.*s", event->topic_len, event->topic);
        sprintf(data, "%.*s", event->data_len, event->data);

        if (strcmp(topic, TOPIC_MODE) == 0) {
            if (strcmp(data, "auto") == 0) {
                current_mode = MODE_AUTO;
                just_switched_to_auto = true;
                ESP_LOGI(TAG, "Modo: AUTO");
            } else if (strcmp(data, "manual") == 0) {
                current_mode = MODE_MANUAL;
                ESP_LOGI(TAG, "Modo: MANUAL");
            }
        } else if (strcmp(topic, TOPIC_MANUAL) == 0 && current_mode == MODE_MANUAL) {
            if (strcmp(data, "left") == 0) set_servo_pwm(current_pwm + ANGLE_STEP);
            else if (strcmp(data, "right") == 0) set_servo_pwm(current_pwm - ANGLE_STEP);
        }
    }
}

void wifi_init() {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = { .broker.address.uri = MQTT_BROKER_URI };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

// --- LOOP PRINCIPAL ---
void control_task(void *pvParameter) {
    int state_auto = 0; // 0 = GSU 1, 1 = GSU 2
    while (1) {
        if (current_mode == MODE_AUTO) {
            if (just_switched_to_auto) { state_auto = 0; just_switched_to_auto = false; }
            
            if (state_auto == 0) {
                set_servo_pwm(ANGLE_LEFT_GSU);
                esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "1", 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
                state_auto = 1;
            } else {
                set_servo_pwm(ANGLE_RIGHT_GSU);
                esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "2", 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
                state_auto = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void) {
    servo_init();
    set_servo_pwm(ANGLE_LEFT_GSU);
    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(5000)); // Espera conexión WiFi
    mqtt_app_start();
    xTaskCreate(&control_task, "control_task", 4096, NULL, 5, NULL);
}
```

---

## 5. Compilación y Ejecución

Sigue estos pasos en tu Terminal.

1.  **Preparar Entorno:** Carga las variables de entorno de ESP-IDF.
    ```bash
    . $HOME/esp/esp-idf/export.sh
    ```

2.  **Navegar al Proyecto:**
    ```bash
    cd ruta/a/gsu_camera_control
    ```
    *(Asegúrate de estar en la carpeta raíz que contiene el primer `CMakeLists.txt`).*

3.  **Configurar Target (Chip):**
    ```bash
    idf.py set-target esp32
    ```

4.  **Compilar, Subir y Monitorear:**
    Conecta la ESP32 mediante USB.
    ```bash
    idf.py build flash monitor
    ```
    *   *Build:* Compila el código.
    *   *Flash:* Sube el binario a la placa.
    *   *Monitor:* Abre la consola serial para ver los logs (`ESP_LOGI`). Para salir del monitor usa `Ctrl + ]`.

---

## 6. Referencia API MQTT

Para controlar el sistema desde la App Móvil o pruebas manuales:

| Tópico | Acción | Payload (Texto) | Descripción |
| :--- | :--- | :--- | :--- |
| `gsu/control/mode` | Publicar | `auto` | Activa rutina automática (GSU 1 -> 5s -> GSU 2). Resetea posición al inicio. |
| `gsu/control/mode` | Publicar | `manual` | Detiene el automático. Mantiene la última posición y espera órdenes. |
| `gsu/control/manual`| Publicar | `left` | Mueve la cámara un paso a la izquierda (solo en modo Manual). |
| `gsu/control/manual`| Publicar | `right` | Mueve la cámara un paso a la derecha (solo en modo Manual). |
| `gsu/data/status` | Suscribir | `1` | Indica que la cámara está enfocando el **GSU Izquierdo**. |
| `gsu/data/status` | Suscribir | `2` | Indica que la cámara está enfocando el **GSU Derecho**. |