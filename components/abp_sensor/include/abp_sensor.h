#ifndef ABP_SENSOR_H
#define ABP_SENSOR_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

<<<<<<< HEAD
#define ABP_I2C_ADDRESS 0x28

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    int offset;
} abp_sensor_t;

/**
 * @brief Initialize ABP sensor on given I2C pins
 */
esp_err_t abp_sensor_init(abp_sensor_t *sensor, i2c_port_t port,
                          gpio_num_t scl, gpio_num_t sda);

/**
 * @brief Configure sensor (no-op for Honeywell ABP)
 */
esp_err_t abp_sensor_configure(abp_sensor_t *sensor);

/**
 * @brief Read raw value from sensor
 */
esp_err_t abp_sensor_read(abp_sensor_t *sensor, int *value);

/**
 * @brief Calibrate sensor and store offset
 */
esp_err_t abp_sensor_calibrate(abp_sensor_t *sensor, int samples);

/**
 * @brief Check for communication errors
 */
esp_err_t abp_sensor_error_check(abp_sensor_t *sensor);

/**
 * @brief Deinitialize sensor and free I2C resources
 */
void abp_sensor_deinit(abp_sensor_t *sensor);

=======
#define ABP_DEFAULT_ADDRESS 0x28

typedef struct {
    i2c_port_t port;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    int16_t offset;
    gpio_num_t sda;
    gpio_num_t scl;
} abp_sensor_t;

esp_err_t abp_sensor_init(abp_sensor_t *sensor, i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
esp_err_t abp_sensor_configure(abp_sensor_t *sensor);
esp_err_t abp_sensor_calibrate(abp_sensor_t *sensor, uint8_t samples);
esp_err_t abp_sensor_read_raw(abp_sensor_t *sensor, int16_t *raw);
esp_err_t abp_sensor_error_check(abp_sensor_t *sensor);

>>>>>>> main
#ifdef __cplusplus
}
#endif

<<<<<<< HEAD
#endif /* ABP_SENSOR_H */
=======
#endif // ABP_SENSOR_H
>>>>>>> main
