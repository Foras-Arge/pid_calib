#include "drv8308.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "drv8308";
static spi_device_handle_t s_spi;

/** SPI init helper */
static esp_err_t drv8308_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = DRV_SDI_PIN,
        .miso_io_num = DRV_SDO_PIN,
        .sclk_io_num = DRV_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = DRV_SCS_PIN,
        .queue_size = 1,
        .flags = SPI_DEVICE_POSITIVE_CS,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));
    return ESP_OK;
}

/**
 * @brief Initialize the DRV8308 driver and PWM timer
 */
esp_err_t drv8308_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<DRV_DIR_PIN)|(1ULL<<DRV_BRAKE_PIN)|
                        (1ULL<<DRV_RESET_PIN)|(1ULL<<DRV_ENABLE_PIN)|
                        (1ULL<<DRV_PWM_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    ESP_ERROR_CHECK(drv8308_spi_init());

    ledc_timer_config_t tim = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = DRV_DEFAULT_PWM_FREQ,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tim));

    ledc_channel_config_t ch = {
        .gpio_num   = DRV_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    gpio_set_level(DRV_ENABLE_PIN, 1);
    gpio_set_level(DRV_RESET_PIN, 0);
    gpio_set_level(DRV_DIR_PIN, 1);
    gpio_set_level(DRV_BRAKE_PIN, 0);

    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

static esp_err_t write_reg(uint8_t addr, uint16_t data)
{
    uint32_t value = ((uint32_t)addr<<16) | data;
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = &value,
    };
    return spi_device_transmit(s_spi, &t);
}

/**
 * @brief Configure PWM frequency and duty cycle
 */
esp_err_t drv8308_configure(const drv8308_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    /* Example: configure a simple register for PWM mode */
    ESP_ERROR_CHECK(write_reg(0x00, 0x0100));
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0,
                                  cfg->pwm_freq_hz));
    return drv8308_set_duty(cfg->duty_cycle);
}

/**
 * @brief Set PWM duty cycle for the motor
 */
esp_err_t drv8308_set_duty(float duty)
{
    if (duty < 0) duty = 0; if (duty > 100) duty = 100;
    uint32_t val = (uint32_t)(duty / 100.0f * 1023);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, val));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    return ESP_OK;
}

/**
 * @brief Read the driver status register
 */
esp_err_t drv8308_read(drv8308_status_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;
    uint8_t cmd = 0x80; // read fault register
    uint32_t tx = cmd << 16;
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = &tx,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    esp_err_t ret = spi_device_transmit(s_spi, &t);
    if (ret == ESP_OK) {
        status->fault_reg = (t.rx_data[1] << 8) | t.rx_data[2];
    }
    return ret;
}

/**
 * @brief Reset the driver for calibration
 */
esp_err_t drv8308_calibrate(void)
{
    /* simply reset driver */
    gpio_set_level(DRV_RESET_PIN, 1);
    ets_delay_us(1000);
    gpio_set_level(DRV_RESET_PIN, 0);
    return ESP_OK;
}

/**
 * @brief Query the fault register and log errors
 */
esp_err_t drv8308_error_check(void)
{
    drv8308_status_t st;
    ESP_ERROR_CHECK(drv8308_read(&st));
    if (st.fault_reg != 0) {
        ESP_LOGE(TAG, "fault register: 0x%04x", st.fault_reg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

