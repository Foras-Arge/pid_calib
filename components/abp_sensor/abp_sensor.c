#include "abp_sensor.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "abp";

esp_err_t abp_sensor_init(abp_sensor_t *sensor, i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    if (!sensor) return ESP_ERR_INVALID_ARG;
    sensor->port = port;
    sensor->sda = sda;
    sensor->scl = scl;
    i2c_master_bus_config_t buscfg = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&buscfg, &sensor->bus), TAG, "bus");

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ABP_DEFAULT_ADDRESS,
        .scl_speed_hz = 100000
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(sensor->bus, &devcfg, &sensor->dev), TAG, "dev");
    sensor->offset = 0;
    return ESP_OK;
}

esp_err_t abp_sensor_configure(abp_sensor_t *sensor)
{
    // sensor requires no special configuration
    return ESP_OK;
}

esp_err_t abp_sensor_calibrate(abp_sensor_t *sensor, uint8_t samples)
{
    int32_t sum = 0;
    int16_t raw;
    for (uint8_t i=0;i<samples;i++) {
        ESP_RETURN_ON_ERROR(abp_sensor_read_raw(sensor, &raw), TAG, "read");
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    sensor->offset = sum / samples;
    ESP_LOGI(TAG, "offset=%d", sensor->offset);
    return ESP_OK;
}

esp_err_t abp_sensor_read_raw(abp_sensor_t *sensor, int16_t *raw)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_receive(sensor->dev, buf, sizeof(buf), 1000/portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    *raw = ((buf[0] & 0x3F) << 8) | buf[1];
    return ESP_OK;
}

esp_err_t abp_sensor_error_check(abp_sensor_t *sensor)
{
    // device does not provide error status over I2C; stub
    (void)sensor;
    return ESP_OK;
}
