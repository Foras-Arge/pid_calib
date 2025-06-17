#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "drv8308.h"
#include "abp_sensor.h"

static const char *TAG = "app";

static drv8308_t motor;
static abp_sensor_t pressure_sensor;
static float kp = 1.0f, ki = 0.1f, kd = 0.01f;
static float integral = 0, prev_error = 0;
static float setpoint = 50.0f; // target pressure

static void control_task(void *arg)
{
    (void)arg;
    int16_t raw;
    float pressure;
    for(;;) {
        if (abp_sensor_read_raw(&pressure_sensor, &raw) == ESP_OK) {
            pressure = (float)(raw - pressure_sensor.offset) * 0.1f; // example scale
            float error = setpoint - pressure;
            integral += error * 0.01f; // 10ms
            if (integral > 100) integral = 100;
            if (integral < -100) integral = -100;
            float derivative = (error - prev_error)/0.01f;
            float output = kp * error + ki * integral + kd * derivative;
            if (output < 0) output = 0;
            if (output > 100) output = 100;
            drv8308_set_pwm(&motor, output);
            ESP_LOGI(TAG, "sp=%.1f pv=%.1f out=%.1f", setpoint, pressure, output);
            prev_error = error;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    drv8308_init(&motor);
    drv8308_configure(&motor);
    abp_sensor_init(&pressure_sensor, I2C_NUM_0, GPIO_NUM_3, GPIO_NUM_8);
    abp_sensor_configure(&pressure_sensor);
    abp_sensor_calibrate(&pressure_sensor, 50);
    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
}
