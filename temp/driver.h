#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define DRV_DIR_PIN GPIO_NUM_21
#define DRV_BRAKE_PIN GPIO_NUM_41
#define DRV_RESET_PIN GPIO_NUM_48
#define DRV_ENABLE_PIN GPIO_NUM_47

#define DRV_FGOUT_PIN GPIO_NUM_39
#define DRV_FAULTN_PIN GPIO_NUM_40
#define DRV_LOCKN_PIN GPIO_NUM_45

#define DRV_SDI_PIN GPIO_NUM_35
#define DRV_SDO_PIN GPIO_NUM_37
#define DRV_SCLK_PIN GPIO_NUM_36
#define DRV_SCS_PIN GPIO_NUM_38

#define DRV_PWM_PIN GPIO_NUM_42

extern spi_device_handle_t drv8308_handle;

extern bool G_BRAKE;
extern bool G_DIR;
extern bool G_ENABLE;
extern bool G_RESET;

// DRV8308 Registers
extern struct CTRL1_Register G_CTRL1_REG;
extern struct ADVANCE_Register G_ADVANCE_REG;
extern struct COMCTRL1_Register G_COMCTRL1_REG;
extern struct MOD120_Register G_MOD120_REG;
extern struct DRIVE_Register G_DRIVE_REG;
extern struct SPDGAIN_Register G_SPDGAIN_REG;
extern struct FILK1_Register G_FILK1_REG;
extern struct FILK2_Register G_FILK2_REG;
extern struct COMPK1_Register G_COMPK1_REG;
extern struct COMPK2_Register G_COMPK2_REG;
extern struct LOOPGN_Register G_LOOPGN_REG;
extern struct SPEED_Register G_SPEED_REG;
extern struct FAULT_Register G_FAULT_REG;

// GUI Variables
extern unsigned int G_TAB_SELECT;
extern unsigned int G_PWM_MODE;
extern unsigned int G_CLK_MODE;
extern unsigned int G_REG_MODE;

extern bool G_AUTO_WRITE;
extern bool G_READ_ALL;

// Main Functions
void driver_init(void);

void driver_speed_control(float duty_cycle);

void start_motor(void);

void stop_motor(void);
