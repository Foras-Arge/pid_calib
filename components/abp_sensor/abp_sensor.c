#include "abp_sensor.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "abp";

esp_err_t abp_sensor_init(abp_sensor_t *sensor, i2c_port_t port,
                          gpio_num_t scl, gpio_num_t sda)
{
    if (!sensor) return ESP_ERR_INVALID_ARG;
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_APB,
        .i2c_port = port,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&cfg, &sensor->bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ABP_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(sensor->bus, &devcfg, &sensor->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(sensor->bus);
        sensor->bus = NULL;
        return ret;
    }
    sensor->offset = 0;
    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

esp_err_t abp_sensor_configure(abp_sensor_t *sensor)
{
    // No specific configuration needed for basic operation
    return sensor ? ESP_OK : ESP_ERR_INVALID_ARG;
}

esp_err_t abp_sensor_read(abp_sensor_t *sensor, int *value)
{
    if (!sensor || !value) return ESP_ERR_INVALID_ARG;
    uint8_t data[2];
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && ret != ESP_OK; ++attempt) {
        ret = i2c_master_receive(sensor->dev, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read fail: %s", esp_err_to_name(ret));
        return ret;
    }
    int raw = ((data[0] & 0x3F) << 8) | data[1];
    *value = raw - sensor->offset;
    return ESP_OK;
}

esp_err_t abp_sensor_calibrate(abp_sensor_t *sensor, int samples)
{
    if (!sensor) return ESP_ERR_INVALID_ARG;
    int sum = 0, val = 0;
    for (int i=0;i<samples;i++) {
        if (abp_sensor_read(sensor, &val) != ESP_OK) return ESP_FAIL;
        sum += val;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    sensor->offset = sum / samples;
    ESP_LOGI(TAG, "offset %d", sensor->offset);
    return ESP_OK;
}

esp_err_t abp_sensor_error_check(abp_sensor_t *sensor)
{
    int dummy;
    esp_err_t ret = abp_sensor_read(sensor, &dummy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read error");
    }
    return ret;
}

void abp_sensor_deinit(abp_sensor_t *sensor)
{
    if (!sensor) return;
    if (sensor->dev) {
        i2c_master_bus_rm_device(sensor->dev);
        sensor->dev = NULL;
    }
    if (sensor->bus) {
        i2c_del_master_bus(sensor->bus);
        sensor->bus = NULL;
    }
}

