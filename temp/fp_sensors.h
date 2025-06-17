#pragma once

#include <stdio.h>
#include "esp_err.h"
#include "settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FP_FLOW_SCL_IO GPIO_NUM_8
#define FP_FLOW_SDA_IO GPIO_NUM_3

#define FP_PRESS_SCL_IO GPIO_NUM_2
#define FP_PRESS_SDA_IO GPIO_NUM_1

extern device_data_t device_data;
extern device_setting_t device_setting;
extern TaskHandle_t fp_sensors_task_handle;

typedef enum
{
    PRESSURE_SENSOR_INIT_FAILED,
    PRESSURE_BASE_ERROR,
    PRESSURE_SENSOR_ERROR,
    FLOW_SENSOR_INIT_FAILED,
    FLOW_BASE_ERROR,
    FLOW_SENSOR_ERROR,
    FP_SENSORS_INIT_SUCCESS,
} fp_sensors_init_status_t;

void fp_sensors_task(void *pvParameters);
void increase_decrease_pressure(uint16_t final_pressure);
fp_sensors_init_status_t fp_sensors_init();
void deinit_sensors();