#ifndef DRV8308_H
#define DRV8308_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPIO definitions for DRV8308 control pins */
#define DRV_DIR_PIN    GPIO_NUM_21
#define DRV_BRAKE_PIN  GPIO_NUM_41
#define DRV_RESET_PIN  GPIO_NUM_48
#define DRV_ENABLE_PIN GPIO_NUM_47

#define DRV_FGOUT_PIN  GPIO_NUM_39
#define DRV_FAULTN_PIN GPIO_NUM_40
#define DRV_LOCKN_PIN  GPIO_NUM_45

#define DRV_SDI_PIN    GPIO_NUM_35
#define DRV_SDO_PIN    GPIO_NUM_37
#define DRV_SCLK_PIN   GPIO_NUM_36
#define DRV_SCS_PIN    GPIO_NUM_38

#define DRV_PWM_PIN    GPIO_NUM_42

/** DRV8308 device handle */
typedef struct {
    spi_device_handle_t spi;
    bool brake;
    bool dir;
    bool enable;
} drv8308_t;

esp_err_t drv8308_init(drv8308_t *dev);
esp_err_t drv8308_configure(drv8308_t *dev);
esp_err_t drv8308_read_fault(drv8308_t *dev, uint16_t *fault);
esp_err_t drv8308_calibrate(drv8308_t *dev);
esp_err_t drv8308_error_check(drv8308_t *dev);
esp_err_t drv8308_set_pwm(drv8308_t *dev, float duty_cycle);

#ifdef __cplusplus
}
#endif

#endif // DRV8308_H
