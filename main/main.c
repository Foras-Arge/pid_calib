#include "drv8308.h"
#include "abp_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "app";

static abp_sensor_t pressure_sensor;
static drv8308_config_t motor_cfg = {
    .pwm_freq_hz = 16000,
    .duty_cycle = 0.0f,
};

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_err;
} pid_t;

static pid_t pid = { .kp = 1.0f, .ki = 0.1f, .kd = 0.05f };
static float setpoint = 100.0f; // desired pressure units

static void control_task(void *arg)
{
    int pressure = 0;
    for(;;) {
        if (abp_sensor_read(&pressure_sensor, &pressure) == ESP_OK) {
            float error = setpoint - (float)pressure;
            pid.integral += error * 0.01f;
            if (pid.integral > 100) pid.integral = 100;
            if (pid.integral < -100) pid.integral = -100;
            float deriv = (error - pid.prev_err) / 0.01f;
            float output = pid.kp * error + pid.ki * pid.integral + pid.kd * deriv;
            pid.prev_err = error;

            float duty = output; // simple mapping
            if (duty < 0) duty = 0; if (duty > 100) duty = 100;
            drv8308_set_duty(duty);
            ESP_LOGI(TAG, "SP %.1f PV %d OUT %.1f", setpoint, pressure, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(drv8308_init());
    ESP_ERROR_CHECK(drv8308_configure(&motor_cfg));

    ESP_ERROR_CHECK(abp_sensor_init(&pressure_sensor, I2C_NUM_0,
                                    GPIO_NUM_2, GPIO_NUM_1));
    ESP_ERROR_CHECK(abp_sensor_configure(&pressure_sensor));
    ESP_ERROR_CHECK(abp_sensor_calibrate(&pressure_sensor, 50));

    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
}

