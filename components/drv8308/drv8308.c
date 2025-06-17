#include "drv8308.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "drv8308";

/* SPI transaction helper */
static esp_err_t drv8308_write_bytes(drv8308_t *dev, const uint8_t *data, size_t len)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = len * 8
    };
    memcpy(t.tx_data, data, len);
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t drv8308_read_reg(drv8308_t *dev, uint8_t addr, uint16_t *val)
{
    uint8_t tx[3] = {0x80 | addr, 0, 0};
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 24,
        .rxlength = 16
    };
    memcpy(t.tx_data, tx, sizeof(tx));
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret == ESP_OK) {
        *val = ((uint16_t)t.rx_data[0] << 8) | t.rx_data[1];
    }
    return ret;
}

esp_err_t drv8308_init(drv8308_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    spi_bus_config_t buscfg = {
        .mosi_io_num = DRV_SDI_PIN,
        .miso_io_num = DRV_SDO_PIN,
        .sclk_io_num = DRV_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "bus init");

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = DRV_SCS_PIN,
        .queue_size = 3,
        .flags = SPI_DEVICE_POSITIVE_CS
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &dev->spi), TAG, "add device");

    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<DRV_DIR_PIN) | (1ULL<<DRV_BRAKE_PIN) |
                        (1ULL<<DRV_RESET_PIN) | (1ULL<<DRV_ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io);

    gpio_set_level(DRV_ENABLE_PIN, 1);
    gpio_set_level(DRV_RESET_PIN, 0);
    gpio_set_level(DRV_DIR_PIN, 1);
    gpio_set_level(DRV_BRAKE_PIN, 0);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 16000
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch = {
        .gpio_num = DRV_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ch);
    dev->brake = false;
    dev->dir = true;
    dev->enable = true;
    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

esp_err_t drv8308_configure(drv8308_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    uint8_t buf[3] = {0x00, 0x90, 0x01};
    ESP_RETURN_ON_ERROR(drv8308_write_bytes(dev, buf, sizeof(buf)), TAG, "cfg");
    return ESP_OK;
}

esp_err_t drv8308_set_pwm(drv8308_t *dev, float duty)
{
    uint32_t value = (uint32_t)(duty * ((1<<13)-1) / 100.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    return ESP_OK;
}

esp_err_t drv8308_read_fault(drv8308_t *dev, uint16_t *fault)
{
    return drv8308_read_reg(dev, 0x0C, fault);
}

esp_err_t drv8308_calibrate(drv8308_t *dev)
{
    // placeholder for calibration sequence
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t drv8308_error_check(drv8308_t *dev)
{
    uint16_t fault;
    ESP_RETURN_ON_ERROR(drv8308_read_fault(dev, &fault), TAG, "read fault");
    if (fault) {
        ESP_LOGW(TAG, "fault register: 0x%04X", fault);
        return ESP_FAIL;
    }
    return ESP_OK;
}
