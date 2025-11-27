
/*
 * GSU Thermal Camera Control - FULL INTEGRATION (PID + MQTT + WIFI)
 * 
 * Requisito: Hack de hardware en servo para feedback en ADC_PIN.
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
#include "driver/adc.h"
#include "mqtt_client.h"

// --- CONFIGURACIÓN DE RED ---
#define WIFI_SSID       "tenda2"
#define WIFI_PASS       "12344321"
#define MQTT_BROKER_URI "mqtt://192.168.0.5"

// --- CONFIGURACIÓN HARDWARE ---
#define SERVO_PIN       18
#define ADC_PIN         ADC1_CHANNEL_6  // GPIO 34
#define SERVO_FREQ      50

// --- PARÁMETROS DE CONTROL ---
#define ANGLE_LEFT      125.0f
#define ANGLE_RIGHT     55.0f
#define MANUAL_STEP     5.0f    // Grados por paso en manual
#define SCAN_TIME_MS    5000

// --- ESTRUCTURA PID ---
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} pid_params_t;

// Variables Globales Compartidas
static const char *TAG = "GSU_PID_SYS";
volatile float target_setpoint = ANGLE_LEFT; // Objetivo actual (Comunicación entre tareas)
volatile float current_position = 0.0f;      // Solo para monitoreo
int current_mode = 0; // 0=AUTO, 1=MANUAL

pid_params_t pid = { .Kp = 1.2, .Ki = 0.1, .Kd = 0.05, .integral=0, .prev_error=0 };
esp_mqtt_client_handle_t mqtt_client;

// --- FUNCIONES HARDWARE (ADC / PWM) ---
void hardware_init() {
    // PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz    = SERVO_FREQ, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_PIN, .duty = 0, .hpoint = 0
    };
    ledc_channel_config(&ledc_ch);

    // ADC (Feedback)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_11);
}

float read_angle_sensor() {
    int raw = adc1_get_raw(ADC_PIN);
    // Calibración simple: Asumiendo 0-3.3V = 0-180 grados (AJUSTAR SEGÚN REALIDAD)
    return ((float)raw / 4095.0f) * 180.0f; 
}

void set_motor_effort(float effort) {
    // effort es la señal de corrección. 
    // Mapeamos esto al PWM del servo (500-2400us)
    // Asumimos que 1500us es el punto neutro de fuerza, sumamos el esfuerzo PID
    // Nota: En servos RC, la posición es proporcional al PWM. 
    // Aquí el PID calcula la posición PWM directa.
    
    // Convertir salida PID (us) a Duty Cycle
    if (effort < 500) effort = 500;
    if (effort > 2400) effort = 2400;
    
    uint32_t duty = (uint32_t)((effort / 20000.0f) * 8191.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// --- TAREA PID (CORRE A 50HZ) ---
void pid_task(void *pvParam) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms loop
    float dt = 0.02;

    while (1) {
        // 1. Leer Sensor
        current_position = read_angle_sensor();

        // 2. Calcular Error
        float error = target_setpoint - current_position;

        // 3. PID
        float P = pid.Kp * error;
        pid.integral += error * dt;
        
        // Anti-windup simple
        if (pid.integral > 100) pid.integral = 100;
        if (pid.integral < -100) pid.integral = -100;

        float I = pid.Ki * pid.integral;
        float D = pid.Kd * (error - pid.prev_error) / dt;
        
        float output = P + I + D;
        pid.prev_error = error;

        // 4. Mapear Salida PID a PWM (us)
        // Mapeo lineal simple: (Grados -> us) + Corrección
        // Base teórica: 0deg=500us, 180deg=2400us -> ~10.5us por grado
        float feed_forward = 500.0f + (target_setpoint * 10.55f); 
        float final_signal = feed_forward + output;

        // 5. Aplicar
        set_motor_effort(final_signal);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- LOGICA DE CONTROL (AUTO/MANUAL) ---
void logic_task(void *pvParam) {
    bool scan_left = true;
    while(1) {
        if (current_mode == 0) { // AUTO
            if (scan_left) {
                target_setpoint = ANGLE_LEFT;
                esp_mqtt_client_publish(mqtt_client, "gsu/data/status", "1", 0, 0, 0);
            } else {
                target_setpoint = ANGLE_RIGHT;
                esp_mqtt_client_publish(mqtt_client, "gsu/data/status", "2", 0, 0, 0);
            }
            scan_left = !scan_left;
            vTaskDelay(pdMS_TO_TICKS(SCAN_TIME_MS));
        } else {
            // En manual no hacemos nada aquí, esperamos MQTT
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- MQTT CALLBACKS ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        esp_mqtt_client_subscribe(mqtt_client, "gsu/control/mode", 0);
        esp_mqtt_client_subscribe(mqtt_client, "gsu/control/manual", 0);
    } else if (event_id == MQTT_EVENT_DATA) {
        char topic[64]; sprintf(topic, "%.*s", event->topic_len, event->topic);
        char data[64];  sprintf(data, "%.*s", event->data_len, event->data);

        if (strcmp(topic, "gsu/control/mode") == 0) {
            if (strcmp(data, "auto") == 0) current_mode = 0;
            else if (strcmp(data, "manual") == 0) current_mode = 1;
        } 
        else if (strcmp(topic, "gsu/control/manual") == 0 && current_mode == 1) {
            // AQUÍ CAMBIAMOS EL SETPOINT, EL PID SE ENCARGA DE MOVERLO
            if (strcmp(data, "left") == 0) target_setpoint += MANUAL_STEP;
            else if (strcmp(data, "right") == 0) target_setpoint -= MANUAL_STEP;
            
            // Límites de seguridad
            if (target_setpoint > 180) target_setpoint = 180;
            if (target_setpoint < 0) target_setpoint = 0;
            
            ESP_LOGI(TAG, "Manual Setpoint: %.1f", target_setpoint);
        }
    }
}

// --- INICIO WIFI/MQTT (BOILERPLATE) ---
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
}

void wifi_init() {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
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

void app_main(void) {
    hardware_init();
    wifi_init();
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar WiFi
    mqtt_app_start();

    // TAREA 1: PID (Alta prioridad, corre rápido)
    xTaskCreate(pid_task, "pid_task", 4096, NULL, 5, NULL);

    // TAREA 2: Lógica de Negocio (Prioridad normal)
    xTaskCreate(logic_task, "logic_task", 4096, NULL, 4, NULL);
}