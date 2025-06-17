#include "driver.h"
#include "registers.h"
#include "string.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "settings.h"

#define TAG "DRV8308_SPI"

#define RegWrite 0x00
#define RegRead 0x80

bool G_BRAKE = false;
bool G_DIR = false;
bool G_ENABLE = true;
bool G_RESET = false;

spi_device_handle_t drv8308_handle;

// DRV8308 Registers
struct CTRL1_Register G_CTRL1_REG;
struct ADVANCE_Register G_ADVANCE_REG;
struct COMCTRL1_Register G_COMCTRL1_REG;
struct MOD120_Register G_MOD120_REG;
struct DRIVE_Register G_DRIVE_REG;
struct SPDGAIN_Register G_SPDGAIN_REG;
struct FILK1_Register G_FILK1_REG;
struct FILK2_Register G_FILK2_REG;
struct COMPK1_Register G_COMPK1_REG;
struct COMPK2_Register G_COMPK2_REG;
struct LOOPGN_Register G_LOOPGN_REG;
struct SPEED_Register G_SPEED_REG;
struct FAULT_Register G_FAULT_REG;

// GUI Variables
unsigned int G_TAB_SELECT = 0;
unsigned int G_PWM_MODE = 0;
unsigned int G_CLK_MODE = 1;
unsigned int G_REG_MODE = 1;

bool G_AUTO_WRITE = true;
bool G_READ_ALL = true;

uint16_t GSizeW = 3;
uint16_t GSizeR = 2;

void init_gpio()
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DRV_SCS_PIN) | (1ULL << DRV_RESET_PIN) |
                           (1ULL << DRV_ENABLE_PIN) | (1ULL << DRV_DIR_PIN) |
                           (1ULL << DRV_BRAKE_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << DRV_FAULTN_PIN) | (1ULL << DRV_FGOUT_PIN) |
                           (1ULL << DRV_LOCKN_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
#if DRIVER_DEBUG
    ESP_LOGI(TAG, "GPIO Initialized");
#endif

    // Set default levels for output pins
    gpio_set_level(DRV_SCS_PIN, 1);
    gpio_set_level(DRV_RESET_PIN, 0);
    gpio_set_level(DRV_ENABLE_PIN, 1);
    gpio_set_level(DRV_DIR_PIN, 1);
    gpio_set_level(DRV_BRAKE_PIN, 0);
}

void update_gpio()
{
    // BRAKE kontrolü
    gpio_set_level(DRV_BRAKE_PIN, G_BRAKE ? 1 : 0);

    // DIR kontrolü
    gpio_set_level(DRV_DIR_PIN, 1);

    // ENABLE kontrolü
    gpio_set_level(DRV_ENABLE_PIN, G_ENABLE);

    // RESET kontrolü
    gpio_set_level(DRV_RESET_PIN, G_RESET ? 1 : 0);
}

void set_default_registers()
{
    // CTRL1 Register
    G_CTRL1_REG.Address = 0x00;
    G_CTRL1_REG.AG_SETPT = 0x9;
    G_CTRL1_REG.ENPOL = 0x0;
    G_CTRL1_REG.DIRPOL = 0x1;
    G_CTRL1_REG.BRKPOL = 0x0;
    G_CTRL1_REG.SYNRECT = 0x1;
    G_CTRL1_REG.PWMF = 0x0;
    G_CTRL1_REG.SPDMODE = 0x01;
    G_CTRL1_REG.FGSEL = 0x0;
    G_CTRL1_REG.BRKMOD = 0x0;
    G_CTRL1_REG.RETRY = 0x1;

    // ADVANCE Register
    G_ADVANCE_REG.Address = 0x01;
    G_ADVANCE_REG.ADVANCE = 90;

    // COMCTRL1 Register
    G_COMCTRL1_REG.Address = 0x02;
    G_COMCTRL1_REG.SPDREVS = 0x03;
    G_COMCTRL1_REG.MINSPD = 0xB4;

    // MOD120 Register
    G_MOD120_REG.Address = 0x03;
    G_MOD120_REG.BASIC = 0x0;
    G_MOD120_REG.SPEEDTH = 0x6;
    G_MOD120_REG.MOD120 = 0x800;

    // DRIVE Register
    G_DRIVE_REG.Address = 0x04;
    G_DRIVE_REG.LRTIME = 0x0;
    G_DRIVE_REG.HALLRST = 0x0;
    G_DRIVE_REG.DELAY = 0x0;
    G_DRIVE_REG.AUTOADV = 0x0;
    G_DRIVE_REG.AUTOGN = 0x1;
    G_DRIVE_REG.ENSINE = 0x0;
    G_DRIVE_REG.TDRIVE = 0x1;
    G_DRIVE_REG.DTIME = 0x0;
    G_DRIVE_REG.IDRIVE = 0x0;

    // SPDGAIN Register
    G_SPDGAIN_REG.Address = 0x05;
    G_SPDGAIN_REG.INTCLK = 0x3;
    G_SPDGAIN_REG.SPDGAIN = 0x007;

    // FILK1 Register
    G_FILK1_REG.Address = 0x06;
    G_FILK1_REG.HALLPOL = 0x01;
    G_FILK1_REG.BYPFILT = 0x0;
    G_FILK1_REG.FILK1 = 0x4B0;

    // FILK2 Register
    G_FILK2_REG.Address = 0x07;
    G_FILK2_REG.FILK2 = 0x3B6;

    // COMPK1 Register
    G_COMPK1_REG.Address = 0x08;
    G_COMPK1_REG.BYPCOMP = 0x0;
    G_COMPK1_REG.COMPK1 = 0x12C;

    // COMPK2 Register
    G_COMPK2_REG.Address = 0x09;
    G_COMPK2_REG.AA_SETPT = 0x0;
    G_COMPK2_REG.COMPK2 = 0x258;

    // LOOPGN Register
    G_LOOPGN_REG.Address = 0x0A;
    G_LOOPGN_REG.OCPDEG = 0x3;
    G_LOOPGN_REG.OCPTH = 0x3;
    G_LOOPGN_REG.OVTH = 0x0;
    G_LOOPGN_REG.VREF_EN = 0x0;
    G_LOOPGN_REG.LOOPGN = 0x064;

    // SPEED Register
    G_SPEED_REG.Address = 0x0B;
    G_SPEED_REG.SPEED = 0x5DC;

    // FAULT Register
    G_FAULT_REG.Address = 0x0C;
    G_FAULT_REG.RLOCK = 0x0;
    G_FAULT_REG.VMOV = 0x0;
    G_FAULT_REG.CPFAIL = 0x0;
    G_FAULT_REG.UVLO = 0x0;
    G_FAULT_REG.OTS = 0x0;
    G_FAULT_REG.CPOC = 0x0;
    G_FAULT_REG.OCP = 0x0;
}

void init_spi2(void)
{
    esp_err_t ret;

    // SPI Bus Konfigürasyonu
    spi_bus_config_t buscfg = {
        .mosi_io_num = DRV_SDI_PIN,
        .miso_io_num = DRV_SDO_PIN,
        .sclk_io_num = DRV_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092, // Adjust based on your ESP32 variant
    };
    // SPI Cihaz Konfigürasyonu
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,   // SPI frekansı: 1 MHz (BaudRatePrescaler ile eşdeğer)
        .mode = 0,                   // SPI Modu 0 (CPOL=0, CPHA=0)
        .spics_io_num = DRV_SCS_PIN, // CS pini (NSS karşılığı)
        .flags = SPI_DEVICE_POSITIVE_CS,
        .queue_size = 7, // Kuyruk boyutu
        .input_delay_ns = 10,
    };

    // SPI Bus Başlatma
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
#if DRIVER_DEBUG
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
#endif
        return;
    }

    // SPI Cihazı SPI2'ye Bağlama
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &drv8308_handle);
    if (ret != ESP_OK)
    {
#if DRIVER_DEBUG
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
#endif
        return;
    }
#if DRIVER_DEBUG
    ESP_LOGI(TAG, "SPI2 initialized successfully");
#endif
}

void write_spi(spi_device_handle_t drv8308_handle, uint8_t *pData)
{
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.length = GSizeW * 8;
    transaction.rxlength = 0;

    transaction.flags = SPI_TRANS_USE_TXDATA;
    memcpy(transaction.tx_data, pData, GSizeW);

    spi_device_transmit(drv8308_handle, &transaction);
}

unsigned int read_spi(spi_device_handle_t drv8308_handle, uint8_t *pData)
{
    unsigned int return_val = 0;
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.flags |= SPI_TRANS_USE_RXDATA;
    transaction.flags |= SPI_TRANS_USE_TXDATA;
    memcpy(transaction.tx_data, pData, GSizeW);
#if DRIVER_DEBUG
    printf("Gönderilen veri: ");
#endif
    for (uint8_t i = 0; i < GSizeW; i++)
    {
#if DRIVER_DEBUG
        printf("%#04x ", transaction.tx_data[i]);
#endif
    }
#if DRIVER_DEBUG
    printf("\n");
#endif

    transaction.length = GSizeW * 8;
    transaction.rxlength = GSizeR * 8;

    spi_device_transmit(drv8308_handle, &transaction);
#if DRIVER_DEBUG
    printf("Okunan veri: ");
#endif
    for (uint8_t i = 0; i < GSizeR; i++)
    {
        uint8_t received_byte = transaction.rx_data[i];
#if DRIVER_DEBUG
        printf("%#04x ", received_byte);
#endif
        return_val |= received_byte << (8 * (GSizeR - i - 1));
    }
#if DRIVER_DEBUG
    printf("\n");
#endif

    return return_val;
}

void write_all_registers()
{
    unsigned char byte0 = 0x00;
    unsigned char byte1 = 0x00;
    unsigned char byte2 = 0x00;
    unsigned char TRANSMIT_DATA[3];

    byte0 = RegWrite | G_CTRL1_REG.Address;
    byte1 = (G_CTRL1_REG.AG_SETPT << 4) | (G_CTRL1_REG.ENPOL << 3) | (G_CTRL1_REG.DIRPOL << 2) | (G_CTRL1_REG.BRKPOL << 1) | (G_CTRL1_REG.SYNRECT);
    byte2 = (G_CTRL1_REG.PWMF << 6) | (G_CTRL1_REG.SPDMODE << 4) | (G_CTRL1_REG.FGSEL << 2) | (G_CTRL1_REG.BRKMOD << 1) | (G_CTRL1_REG.RETRY);
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_ADVANCE_REG.Address;
    byte1 = 0x00;
    byte2 = G_ADVANCE_REG.ADVANCE;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_COMCTRL1_REG.Address;
    byte1 = G_COMCTRL1_REG.SPDREVS;
    byte2 = G_COMCTRL1_REG.MINSPD;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_MOD120_REG.Address;
    byte1 = (G_MOD120_REG.BASIC << 7) | (G_MOD120_REG.SPEEDTH << 4) | (G_MOD120_REG.MOD120 >> 8);
    byte2 = G_MOD120_REG.MOD120;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_DRIVE_REG.Address;
    byte1 = (G_DRIVE_REG.LRTIME << 6) | (G_DRIVE_REG.HALLRST << 4) | (G_DRIVE_REG.DELAY << 3) | (G_DRIVE_REG.AUTOADV << 2) | (G_DRIVE_REG.AUTOGN << 1) | (G_DRIVE_REG.ENSINE);
    byte2 = (G_DRIVE_REG.TDRIVE << 6) | (G_DRIVE_REG.DTIME << 3) | (G_DRIVE_REG.IDRIVE);
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_SPDGAIN_REG.Address;
    byte1 = (G_SPDGAIN_REG.INTCLK << 4) | (G_SPDGAIN_REG.SPDGAIN >> 8);
    byte2 = G_SPDGAIN_REG.SPDGAIN;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);

    byte0 = RegWrite | G_FILK1_REG.Address;
    byte1 = (G_FILK1_REG.HALLPOL << 7) | (G_FILK1_REG.BYPFILT << 4) | (G_FILK1_REG.FILK1 >> 8);
    byte2 = G_FILK1_REG.FILK1;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_FILK2_REG.Address;
    byte1 = (G_FILK2_REG.FILK2 >> 8);
    byte2 = G_FILK2_REG.FILK2;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);

    byte0 = RegWrite | G_COMPK1_REG.Address;
    byte1 = (G_COMPK1_REG.BYPCOMP << 4) | (G_COMPK1_REG.COMPK1 >> 8);
    byte2 = G_COMPK1_REG.COMPK1;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);

    byte0 = RegWrite | G_COMPK2_REG.Address;
    byte1 = (G_COMPK2_REG.AA_SETPT << 4) | (G_COMPK2_REG.COMPK2 >> 8);
    byte2 = G_COMPK2_REG.COMPK2;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);
    byte0 = RegWrite | G_LOOPGN_REG.Address;
    byte1 = (G_LOOPGN_REG.OCPDEG << 6) | (G_LOOPGN_REG.OCPTH << 4) | (G_LOOPGN_REG.OVTH << 3) | (G_LOOPGN_REG.VREF_EN << 2) | (G_LOOPGN_REG.LOOPGN >> 8);
    byte2 = G_LOOPGN_REG.LOOPGN;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;
    write_spi(drv8308_handle, TRANSMIT_DATA);

    byte0 = RegWrite | G_SPEED_REG.Address;
    byte1 = (G_SPEED_REG.SPEED >> 8);
    byte2 = G_SPEED_REG.SPEED;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    if (G_READ_ALL == true)
    {
        write_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
        printf("G_CTRL1_REG.AG_SETPT: %#04x\n", G_CTRL1_REG.AG_SETPT);
        printf("G_CTRL1_REG.ENPOL: %#04x\n", G_CTRL1_REG.ENPOL);
        printf("G_CTRL1_REG.DIRPOL: %#04x\n", G_CTRL1_REG.DIRPOL);
        printf("G_CTRL1_REG.BRKPOL: %#04x\n", G_CTRL1_REG.BRKPOL);
        printf("G_CTRL1_REG.SYNRECT: %#04x\n", G_CTRL1_REG.SYNRECT);
        printf("G_CTRL1_REG.PWMF: %#04x\n", G_CTRL1_REG.PWMF);
        printf("G_CTRL1_REG.SPDMODE: %#04x\n", G_CTRL1_REG.SPDMODE);
        printf("G_CTRL1_REG.FGSEL: %#04x\n", G_CTRL1_REG.FGSEL);
        printf("G_CTRL1_REG.BRKMOD: %#04x\n", G_CTRL1_REG.BRKMOD);
        printf("G_CTRL1_REG.RETRY: %#04x\n", G_CTRL1_REG.RETRY);
        printf("G_ADVANCE_REG.ADVANCE: %#04x\n", G_ADVANCE_REG.ADVANCE);
        printf("G_COMCTRL1_REG.SPDREVS: %#04x\n", G_COMCTRL1_REG.SPDREVS);
        printf("G_COMCTRL1_REG.MINSPD: %#04x\n", G_COMCTRL1_REG.MINSPD);
        printf("G_MOD120_REG.BASIC: %#04x\n", G_MOD120_REG.BASIC);
        printf("G_MOD120_REG.SPEEDTH: %#04x\n", G_MOD120_REG.SPEEDTH);
        printf("G_MOD120_REG.MOD120: %#04x\n", G_MOD120_REG.MOD120);
        printf("G_DRIVE_REG.LRTIME: %#04x\n", G_DRIVE_REG.LRTIME);
        printf("G_DRIVE_REG.HALLRST: %#04x\n", G_DRIVE_REG.HALLRST);
        printf("G_DRIVE_REG.DELAY: %#04x\n", G_DRIVE_REG.DELAY);
        printf("G_DRIVE_REG.AUTOADV: %#04x\n", G_DRIVE_REG.AUTOADV);
        printf("G_DRIVE_REG.AUTOGN: %#04x\n", G_DRIVE_REG.AUTOGN);
        printf("G_DRIVE_REG.ENSINE: %#04x\n", G_DRIVE_REG.ENSINE);
        printf("G_DRIVE_REG.TDRIVE: %#04x\n", G_DRIVE_REG.TDRIVE);
        printf("G_DRIVE_REG.DTIME: %#04x\n", G_DRIVE_REG.DTIME);
        printf("G_DRIVE_REG.IDRIVE: %#04x\n", G_DRIVE_REG.IDRIVE);
        printf("G_SPDGAIN_REG.INTCLK: %#04x\n", G_SPDGAIN_REG.INTCLK);
        printf("G_SPDGAIN_REG.SPDGAIN: %#04x\n", G_SPDGAIN_REG.SPDGAIN);
        printf("G_FILK1_REG.HALLPOL: %#04x\n", G_FILK1_REG.HALLPOL);
        printf("G_FILK1_REG.BYPFILT: %#04x\n", G_FILK1_REG.BYPFILT);
        printf("G_FILK1_REG.FILK1: %#04x\n", G_FILK1_REG.FILK1);
        printf("G_FILK2_REG.FILK2: %#04x\n", G_FILK2_REG.FILK2);
        printf("G_COMPK1_REG.BYPCOMP: %#04x\n", G_COMPK1_REG.BYPCOMP);
        printf("G_COMPK1_REG.COMPK1: %#04x\n", G_COMPK1_REG.COMPK1);
        printf("G_COMPK2_REG.AA_SETPT: %#04x\n", G_COMPK2_REG.AA_SETPT);
        printf("G_COMPK2_REG.COMPK2: %#04x\n", G_COMPK2_REG.COMPK2);
#endif
    }
}

void read_all_registers()
{
    unsigned char byte0 = 0x00;
    unsigned char byte1 = 0x00;
    unsigned char byte2 = 0x00;
    static unsigned int readData = 0;
    unsigned char TRANSMIT_DATA[3];

    byte0 = RegRead | G_CTRL1_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_CTRL1_REG.AG_SETPT = ((readData >> 12) & 0x000F);
    G_CTRL1_REG.ENPOL = ((readData >> 11) & 0x0001);
    G_CTRL1_REG.DIRPOL = ((readData >> 10) & 0x0001);
    G_CTRL1_REG.BRKPOL = ((readData >> 9) & 0x0001);
    G_CTRL1_REG.SYNRECT = ((readData >> 8) & 0x0001);
    G_CTRL1_REG.PWMF = ((readData >> 6) & 0x0003);
    G_CTRL1_REG.SPDMODE = ((readData >> 4) & 0x0003);
    G_CTRL1_REG.FGSEL = ((readData >> 2) & 0x0003);
    G_CTRL1_REG.BRKMOD = ((readData >> 1) & 0x0001);
    G_CTRL1_REG.RETRY = (readData & 0x0001);
    byte0 = RegRead | G_ADVANCE_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif
    G_ADVANCE_REG.ADVANCE = (readData & 0x00FF);
    byte0 = RegRead | G_COMCTRL1_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_COMCTRL1_REG.SPDREVS = ((readData >> 8) & 0x00FF);
    G_COMCTRL1_REG.MINSPD = (readData & 0x00FF);

    byte0 = RegRead | G_MOD120_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_MOD120_REG.BASIC = ((readData >> 15) & 0x0001);
    G_MOD120_REG.SPEEDTH = ((readData >> 12) & 0x0007);
    G_MOD120_REG.MOD120 = (readData & 0x0FFF);

    byte0 = RegRead | G_DRIVE_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_DRIVE_REG.LRTIME = ((readData >> 14) & 0x0003);
    G_DRIVE_REG.HALLRST = ((readData >> 12) & 0x0003);
    G_DRIVE_REG.DELAY = ((readData >> 11) & 0x0001);
    G_DRIVE_REG.AUTOADV = ((readData >> 10) & 0x0001);
    G_DRIVE_REG.AUTOGN = ((readData >> 9) & 0x0001);
    G_DRIVE_REG.ENSINE = ((readData >> 8) & 0x0001);
    G_DRIVE_REG.TDRIVE = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.DTIME = ((readData >> 3) & 0x0007);
    G_DRIVE_REG.IDRIVE = (readData & 0x0007);

    byte0 = RegRead | G_SPDGAIN_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_SPDGAIN_REG.INTCLK = ((readData >> 12) & 0x0007);
    G_SPDGAIN_REG.SPDGAIN = (readData & 0x0FFF);

    byte0 = RegRead | G_FILK1_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_FILK1_REG.HALLPOL = ((readData >> 15) & 0x0001);
    G_FILK1_REG.BYPFILT = ((readData >> 12) & 0x0001);
    G_FILK1_REG.FILK1 = (readData & 0x0FFF);

    byte0 = RegRead | G_FILK2_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_FILK2_REG.FILK2 = (readData & 0x0FFF);

    byte0 = RegRead | G_COMPK1_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_COMPK1_REG.BYPCOMP = ((readData >> 12) & 0x0001);
    G_COMPK1_REG.COMPK1 = (readData & 0x0FFF);

    byte0 = RegRead | G_COMPK2_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_COMPK2_REG.AA_SETPT = ((readData >> 12) & 0x000F);
    G_COMPK2_REG.COMPK2 = (readData & 0x0FFF);

    byte0 = RegRead | G_LOOPGN_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_LOOPGN_REG.OCPDEG = ((readData >> 14) & 0x0003);
    G_LOOPGN_REG.OCPTH = ((readData >> 12) & 0x0003);
    G_LOOPGN_REG.OVTH = ((readData >> 11) & 0x0001);
    G_LOOPGN_REG.VREF_EN = ((readData >> 10) & 0x0001);
    G_LOOPGN_REG.LOOPGN = (readData & 0x03FF);

    byte0 = RegRead | G_SPEED_REG.Address;
    TRANSMIT_DATA[0] = byte0;
    TRANSMIT_DATA[1] = byte1;
    TRANSMIT_DATA[2] = byte2;

    readData = read_spi(drv8308_handle, TRANSMIT_DATA);
#if DRIVER_DEBUG
    printf("readData: %#08x\n", readData);
#endif

    G_SPEED_REG.SPEED = (readData & 0x0FFF);
}

void update_registers()
{
    if (G_AUTO_WRITE == true)
    {
        write_all_registers();
    }

    if (G_READ_ALL == true)
    {
        read_all_registers();
        G_READ_ALL = false;
    }
}

// PWM başlatma fonksiyonu
void start_motor()
{
#if DRIVER_DEBUG
    ESP_LOGI("PWM", "Enabling PWM...");
#endif

    // LEDC zamanlayıcı yapılandırması
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 16000,
    };
    ledc_timer_config(&ledc_timer);

    // LEDC kanal yapılandırması
    ledc_channel_config_t ledc_channel = {
        .gpio_num = DRV_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 4095, // Başlangıçta %100 duty cycle (13-bit için max değer)
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
#if DRIVER_DEBUG
    ESP_LOGI("PWM", "PWM Enabled at 100%% duty cycle.");
#endif
}

// PWM durdurma fonksiyonu
void stop_motor()
{
#if DRIVER_DEBUG
    ESP_LOGI("PWM", "Disabling PWM...");
#endif
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Duty cycle'ı sıfıra çekerek PWM'i durdur
}

// Duty cycle güncelleme fonksiyonu
void update_duty_cycle(float duty_cycle)
{
    uint32_t duty_value = (uint32_t)((duty_cycle / 100.0) * 4096); // 13-bit için ölçekleme
#if DRIVER_DEBUG
    ESP_LOGI("PWM", "Updating Duty Cycle to: %.2f%%", duty_cycle);
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// PWM kontrol fonksiyonu
void driver_speed_control(float duty_cycle)
{
#if DRIVER_DEBUG
    ESP_LOGI("PWM", "Updating PWM settings...");
#endif

    update_duty_cycle(duty_cycle);
}

// Function Definitions
void driver_init(void)
{
    init_gpio();

    init_spi2();

    start_motor();

    driver_speed_control(0.0f);

    update_gpio();

    set_default_registers();

    update_registers();
}