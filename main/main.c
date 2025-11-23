
/*
 * GSU Thermal Camera Control - ESP32 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "mqtt_client.h"

// --- CONFIGURACIÓN DE RED Y MQTT ---
#define WIFI_SSID      "l17"
#define WIFI_PASS      "Kalev1501"
#define MQTT_BROKER_URI "mqtt://test.mosquitto.org" // CAMBIAR POR (IP o URL) DEL BROKER MQTT
// Tópicos MQTT
#define TOPIC_MODE     "gsu/control/mode"    // "auto" o "manual"
#define TOPIC_MANUAL   "gsu/control/manual"  // "left", "right"
#define TOPIC_STATUS   "gsu/data/status"     // Publica "1" o "2"

// --- CONFIGURACIÓN SERVO SG90 ---
#define SERVO_PIN      18       // GPIO conectado a la señal del Servo
#define SERVO_FREQ     50       // 50Hz para SG90
#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_RES       LEDC_TIMER_13_BIT // Resolución de 13 bits

// --- GEOMETRÍA DEL PANEO ---
// Asumiendo rango PWM 2.5% a 12.5% para 0-180 grados aprox en 13 bits (8192)
// Calibración típica SG90: 500us a 2400us
// 500us / 20000us * 8192 = ~205
// 2400us / 20000us * 8192 = ~983
#define ANGLE_LEFT_GSU  320  // Aprox 125 grados (Izquierda física, visual derecha según montaje)
#define ANGLE_RIGHT_GSU 160  // Aprox 55 grados
#define ANGLE_STEP      15   // Pasos para modo manual
#define SCAN_INTERVAL_MS 5000 // Tiempo X entre paneos (5 segundos)

// --- VARIABLES GLOBALES ---
static const char *TAG = "GSU_CAM";
typedef enum { MODE_AUTO, MODE_MANUAL } system_mode_t;
system_mode_t current_mode = MODE_AUTO;
int current_pwm = ANGLE_LEFT_GSU; // Posición inicial
bool just_switched_to_auto = true;

static esp_mqtt_client_handle_t mqtt_client;

// --- FUNCIONES SERVO ---
void servo_init() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RES,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_servo_pwm(int duty) {
    // Limites de seguridad para SG90
    if (duty < 100) duty = 100; 
    if (duty > 1000) duty = 1000;
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    current_pwm = duty;
    ESP_LOGI(TAG, "Servo PWM: %d", duty);
}

// --- WIFI & MQTT ---
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconectando al WiFi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi Conectado. IP obtenida.");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT Conectado");
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_MODE, 0);
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_MANUAL, 0);
    } else if (event_id == MQTT_EVENT_DATA) {
        // Procesar datos entrantes
        char topic[64];
        char data[64];
        sprintf(topic, "%.*s", event->topic_len, event->topic);
        sprintf(data, "%.*s", event->data_len, event->data);

        ESP_LOGI(TAG, "Topic: %s, Data: %s", topic, data);

        if (strcmp(topic, TOPIC_MODE) == 0) {
            if (strcmp(data, "auto") == 0) {
                current_mode = MODE_AUTO;
                just_switched_to_auto = true; // Bandera para reiniciar posición
                ESP_LOGI(TAG, "Modo cambiado a AUTO");
            } else if (strcmp(data, "manual") == 0) {
                current_mode = MODE_MANUAL;
                ESP_LOGI(TAG, "Modo cambiado a MANUAL");
            }
        } else if (strcmp(topic, TOPIC_MANUAL) == 0 && current_mode == MODE_MANUAL) {
            if (strcmp(data, "left") == 0) { // Mover a izquierda (aumentar PWM usualmente en servo invertido o viceversa)
                 set_servo_pwm(current_pwm + ANGLE_STEP); 
            } else if (strcmp(data, "right") == 0) {
                 set_servo_pwm(current_pwm - ANGLE_STEP);
            }
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
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

// --- TAREA PRINCIPAL DE CONTROL ---
void control_task(void *pvParameter) {
    int state_auto = 0; // 0 = GSU 1 (Izquierda), 1 = GSU 2 (Derecha)

    while (1) {
        if (current_mode == MODE_AUTO) {
            // Si acabamos de cambiar a auto, forzamos reinicio a GSU 1
            if (just_switched_to_auto) {
                state_auto = 0;
                just_switched_to_auto = false;
            }

            if (state_auto == 0) {
                // Mirar GSU 1 (Izquierda)
                set_servo_pwm(ANGLE_LEFT_GSU);
                // Publicar identificación
                esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "1", 0, 0, 0);
                ESP_LOGI(TAG, "Auto: Mirando GSU 1 (Izq)");
                
                vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS)); // Esperar X tiempo
                state_auto = 1; // Cambiar al siguiente estado
            } else {
                // Mirar GSU 2 (Derecha)
                set_servo_pwm(ANGLE_RIGHT_GSU);
                // Publicar identificación
                esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "2", 0, 0, 0);
                ESP_LOGI(TAG, "Auto: Mirando GSU 2 (Der)");

                vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS)); // Esperar X tiempo
                state_auto = 0; // Volver al inicio
            }
        } else {
            // En modo manual, no hacemos nada en el loop, todo es por eventos MQTT
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Sistema de Paneo GSU...");
    
    // Inicializar Hardware
    servo_init();
    set_servo_pwm(ANGLE_LEFT_GSU); // Posición inicial segura

    // Inicializar Red
    wifi_init();
    // Esperar un poco a que conecte WiFi (en producción usar event groups)
    vTaskDelay(pdMS_TO_TICKS(5000)); 
    mqtt_app_start();

    // Crear tarea de control
    xTaskCreate(&control_task, "control_task", 4096, NULL, 5, NULL);
}