#ifndef DRV8308_H
#define DRV8308_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup drv8308 DRV8308 Motor Driver */

/** \brief GPIO assignments for DRV8308 */
#define DRV_DIR_PIN   GPIO_NUM_21
#define DRV_BRAKE_PIN GPIO_NUM_41
#define DRV_RESET_PIN GPIO_NUM_48
#define DRV_ENABLE_PIN GPIO_NUM_47
#define DRV_FGOUT_PIN GPIO_NUM_39
#define DRV_FAULTN_PIN GPIO_NUM_40
#define DRV_LOCKN_PIN GPIO_NUM_45
#define DRV_SDI_PIN   GPIO_NUM_35
#define DRV_SDO_PIN   GPIO_NUM_37
#define DRV_SCLK_PIN  GPIO_NUM_36
#define DRV_SCS_PIN   GPIO_NUM_38
#define DRV_PWM_PIN   GPIO_NUM_42

/** Default PWM frequency for the driver */
#define DRV_DEFAULT_PWM_FREQ 16000

/** Configuration structure */
typedef struct {
    uint32_t pwm_freq_hz; /*!< PWM frequency */
    float    duty_cycle;  /*!< initial duty cycle 0-100 */
} drv8308_config_t;

/** Status structure */
typedef struct {
    uint16_t fault_reg; /*!< value of fault register */
} drv8308_status_t;

/**
 * @brief Initialize SPI bus and GPIOs for DRV8308
 */
esp_err_t drv8308_init(void);

/**
 * @brief Configure driver registers
 */
esp_err_t drv8308_configure(const drv8308_config_t *cfg);

/**
 * @brief Read status registers
 */
esp_err_t drv8308_read(drv8308_status_t *status);

/**
 * @brief Calibrate driver (reset + default registers)
 */
esp_err_t drv8308_calibrate(void);

/**
 * @brief Check for driver errors
 */
esp_err_t drv8308_error_check(void);

/**
 * @brief Set PWM duty (0-100%%)
 */
esp_err_t drv8308_set_duty(float duty);

#ifdef __cplusplus
}
#endif

#endif /* DRV8308_H */
