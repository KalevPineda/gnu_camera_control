
/*
 * GSU Thermal Camera Control - PID IMPLEMENTATION
 * 
 * Requisito: Pin de Feedback (ADC) conectado al potenciómetro del servo.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_timer.h"

// --- CONFIGURACIÓN SERVO Y PID ---
#define SERVO_PIN       18
#define FEEDBACK_PIN    ADC1_CHANNEL_6  // GPIO 34 (Entrada analógica)
#define SERVO_FREQ      50
#define PID_SAMPLE_TIME 20  // ms (50Hz loop)

// Parámetros del Servo (Calibración PWM)
#define PWM_MIN_US      500
#define PWM_MAX_US      2400
#define ADC_MAX_VAL     4095
#define ANGLE_MAX       180.0

// --- ESTRUCTURA PID ---
typedef struct {
    float Kp;           // Ganancia Proporcional
    float Ki;           // Ganancia Integral
    float Kd;           // Ganancia Derivativa
    float setpoint;     // Objetivo (Grados)
    float integral;     // Acumulador integral
    float prev_error;   // Error anterior
    float out_min;      // Límite salida menor (us)
    float out_max;      // Límite salida mayor (us)
} pid_ctrl_t;

// Sintonización (Tuning) - Valores ejemplo
pid_ctrl_t pid = {
    .Kp = 2.5,          // Fuerza de reacción
    .Ki = 0.1,          // Corrección de error estacionario
    .Kd = 0.05,         // Amortiguamiento
    .setpoint = 90.0,
    .integral = 0.0,
    .prev_error = 0.0,
    .out_min = PWM_MIN_US,  
    .out_max = PWM_MAX_US   
};

static const char *TAG = "PID_CTRL";

// --- INICIALIZACIÓN ---
void hardware_init() {
    // Configurar PWM (LEDC)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz    = SERVO_FREQ,
        .clk_cfg    = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);

    // Configurar ADC (Feedback)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(FEEDBACK_PIN, ADC_ATTEN_DB_11);
}

// --- LECTURA DE SENSOR (PLANT) ---
float read_position() {
    int raw = adc1_get_raw(FEEDBACK_PIN);
    // Mapeo lineal de ADC (0-4095) a Grados (0-180)
    // Nota: Esto requiere calibración real
    return ((float)raw / (float)ADC_MAX_VAL) * ANGLE_MAX;
}

// --- APLICACIÓN DE SALIDA (ACTUATOR) ---
void apply_output(float pwm_us) {
    // Conversión de microsegundos a Duty Cycle (13 bits)
    // Periodo 20ms = 20000us
    // Duty = (us / 20000) * 8191
    uint32_t duty = (uint32_t)((pwm_us / 20000.0) * 8191.0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// --- ALGORITMO PID ---
void pid_compute_task(void *pvParam) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PID_SAMPLE_TIME);
    float dt = PID_SAMPLE_TIME / 1000.0; // Segundos

    while (1) {
        // 1. Medir Variable de Proceso (PV)
        float current_angle = read_position();

        // 2. Calcular Error
        float error = pid.setpoint - current_angle;

        // 3. Término Proporcional
        float P = pid.Kp * error;

        // 4. Término Integral (con Anti-windup)
        pid.integral += error * dt;
        float I = pid.Ki * pid.integral;

        // 5. Término Derivativo
        float derivative = (error - pid.prev_error) / dt;
        float D = pid.Kd * derivative;

        // 6. Calcular Salida Total
        float output = P + I + D;

        // Offset: El PID da corrección, necesitamos el valor base PWM (centro ~1500us)
        // O asumimos que output es el valor absoluto de PWM.
        // En servos, suele ser mejor mapear: 
        // Salida PID (Grados corrección) -> PWM
        // Aquí asumiremos que 'output' es el valor directo en microsegundos si Kp es alto
        // Ojo: Una implementación estándar suma al valor neutro:
        float control_signal = 1500.0 + output; 

        // 7. Saturación (Clamping)
        if (control_signal > pid.out_max) control_signal = pid.out_max;
        if (control_signal < pid.out_min) control_signal = pid.out_min;
        
        // Anti-windup clamping simple
        if (control_signal == pid.out_max || control_signal == pid.out_min) {
            pid.integral -= error * dt; // Deshacer integración si saturó
        }

        // 8. Aplicar al hardware
        apply_output(control_signal);
        pid.prev_error = error;

        // Logs para telemetría
        ESP_LOGI(TAG, "Ref: %.1f | Act: %.1f | Err: %.1f | Out: %.0f", 
                 pid.setpoint, current_angle, error, control_signal);

        // Esperar siguiente ciclo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main() {
    hardware_init();
    
    // Cambiar Setpoint para probar
    pid.setpoint = 125.0; // Grados
    
    xTaskCreate(pid_compute_task, "pid_task", 4096, NULL, 5, NULL);
}