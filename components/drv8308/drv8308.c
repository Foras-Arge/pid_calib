#include "drv8308.h"

#include "esp_log.h"
#include <string.h>

static const char *TAG = "drv8308";

    spi_bus_config_t buscfg = {
        .mosi_io_num = DRV_SDI_PIN,
        .miso_io_num = DRV_SDO_PIN,
        .sclk_io_num = DRV_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = DRV_SCS_PIN,

    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}


        return ESP_FAIL;
    }
    return ESP_OK;
}
