#include "fp_sensors.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "math.h"
#include "esp_system.h"
#include <string.h>

#include "utils.h"
#include "nextion.h"
#include "rtc.h"

#define FLOW_I2C_NUM I2C_NUM_0     // Akış için I2C portu
#define PRESSURE_I2C_NUM I2C_NUM_1 // Basınç için I2C portu
#define I2C_MASTER_FREQ_HZ 100000  // I2C hızı
#define I2C_ADDRESS 0x38           // I2C adresi (8-bit formatında)
#define REPEAT_CALCULATION 5       // Ölçüm ortalaması için tekrar sayısı

static const char *TAG = "Pressure-Flow Sensors";
#if FP_SENSORS_DEBUG
#endif

device_data_t device_data;
device_setting_t device_setting;

TaskHandle_t fp_sensors_task_handle;

// I2C master bus handles
static i2c_master_bus_handle_t flow_bus_handle = NULL;
static i2c_master_bus_handle_t pressure_bus_handle = NULL;
static i2c_master_dev_handle_t flow_dev_handle = NULL;
static i2c_master_dev_handle_t pressure_dev_handle = NULL;

//*****************************************************
// Kalibrasyon (offset) değerleri (sensör boş okuması)
//*****************************************************
static int pressure_base;
static int flow_base;

//*****************************************************
// Dönüşüm ve filtreleme için sabitler
//*****************************************************
#define PRESSURE_CONVERSION_FACTOR 0.095321 // Basınç dönüşüm katsayısı
#define PRESSURE_ALPHA 0.05                 // Basınç için düşük geçiren filtre alfa değeri
#define FLOW_ALPHA 0.1                      // Akış için düşük geçiren filtre alfa değeri

// Düşük geçiren filtre için statik durum değişkenleri
double pressure_filtered = 0;
double flow_filtered = 0;

//*****************************************************
// Dönüşüm ve Filtreleme Fonksiyonları
//*****************************************************

/**
 * @brief Raw basınç değerini, kalibrasyon sonrası (offset düştükten sonra)
 *        fiziksel birime dönüştürür.
 *
 * @param raw_pressure Kalibrasyon uygulanmamış raw basınç değeri
 * @return int Dönüştürülmüş basınç değeri
 */
int convert_pressure(int raw_pressure)
{
    return (int)(PRESSURE_CONVERSION_FACTOR * raw_pressure);
}

/**
 * @brief Raw akış değerini, kalibrasyon sonrası (offset düştükten sonra)
 *        sensöre özgü polinom ile dönüştürür.
 *
 * Bu fonksiyonda, her aralıkta polinomun sabit terimi çıkartılarak
 * f(0)=0 olması sağlanır.
 *
 * @param raw_flow Kalibrasyon uygulanmamış raw akış değeri
 * @return int Dönüştürülmüş akış değeri
 */
int convert_flow(int raw_flow)
{
    // constants from matlab flow_analysis.m
    double p1[] = {2.5419e-06, -0.0017567, 0.4532, 2.5134};
    double p2[] = {4.1106e-07, -0.0005016, 0.24597, 5.7692};
    double p3[] = {-9.0825e-06, 0.063717, 25.757};
    double flow;
    int sign;

    // Handle sign and convert to absolute value
    if (raw_flow < 0)
    {
        sign = -1;
        raw_flow = -raw_flow; // Simplified absolute value calculation
    }
    else
    {
        sign = 1;
    }

    // Determine which polynomial to use based on flow range
    if (raw_flow <= 20)
    {
        flow = p1[0] * raw_flow * raw_flow * raw_flow +
               p1[1] * raw_flow * raw_flow +
               p1[2] * raw_flow +
               p1[3];
    }
    else if (raw_flow <= 450)
    {
        flow = p2[0] * raw_flow * raw_flow * raw_flow +
               p2[1] * raw_flow * raw_flow +
               p2[2] * raw_flow +
               p2[3];
    }
    else
    {
        flow = p3[0] * raw_flow * raw_flow +
               p3[1] * raw_flow +
               p3[2];
    }

    // Apply sign and scaling
    flow = sign * flow * 11;
    return flow;
}

/**
 * @brief Dönüştürülmüş basınç değerine düşük geçiren filtre uygular.
 *
 * @param conv_pressure Dönüştürülmüş basınç değeri
 * @return int Filtrelenmiş basınç değeri
 */
int filter_pressure(int conv_pressure)
{
    pressure_filtered = PRESSURE_ALPHA * conv_pressure + (1.0 - PRESSURE_ALPHA) * pressure_filtered;
    return (int)pressure_filtered;
}

/**
 * @brief Dönüştürülmüş akış değerine düşük geçiren filtre uygular.
 *
 * @param conv_flow Dönüştürülmüş akış değeri
 * @return int Filtrelenmiş akış değeri
 */
int filter_flow(int conv_flow)
{
    static int fout;
    double ef;

    ef = (double)abs(fout - conv_flow) / 600;
    if (ef > 0.95)
    {
        ef = 0.95;
    }
    else if (ef < 0.05)
    {
        ef = 0.05;
    }
    fout = (1 - ef) * fout + ef * conv_flow;

    return fout;
}

/**
 * @brief Raw basınç ve akış değerlerini alır, kalibrasyon (offset düşme),
 *        dönüşüm ve filtre uygulayarak son değerleri verir.
 *
 * @param raw_pressure Raw basınç değeri (örneğin I2C'den okunan)
 * @param raw_flow     Raw akış değeri
 * @param final_pressure Pointer üzerinden elde edilecek son basınç değeri
 * @param final_flow     Pointer üzerinden elde edilecek son akış değeri
 */
void process_sensor_data(int raw_pressure, int raw_flow, int *final_pressure, int *final_flow)
{
    // Kalibrasyon: sensör boşta okunan değeri düşüyoruz
    int calibrated_pressure = raw_pressure - pressure_base;
    int calibrated_flow = raw_flow - flow_base;

    // Dönüşüm: fiziksel birimlere çeviriyoruz
    int conv_pressure = convert_pressure(calibrated_pressure);
    int conv_flow = convert_flow(calibrated_flow);

    // Filtreleme: düşük geçiren filtre uyguluyoruz
    *final_pressure = filter_pressure(conv_pressure);
    *final_flow = filter_flow(conv_flow);
}

//*****************************************************
// I2C Başlatma ve Ölçüm Fonksiyonları
//*****************************************************

esp_err_t init_flow()
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = FLOW_I2C_NUM,
        .scl_io_num = FP_FLOW_SCL_IO,
        .sda_io_num = FP_FLOW_SDA_IO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &flow_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create flow I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(flow_bus_handle, &dev_config, &flow_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add flow I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(flow_bus_handle);
        flow_bus_handle = NULL;
        return ret;
    }

    return ESP_OK;
}

esp_err_t init_pressure()
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PRESSURE_I2C_NUM,
        .scl_io_num = FP_PRESS_SCL_IO,
        .sda_io_num = FP_PRESS_SDA_IO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &pressure_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create pressure I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(pressure_bus_handle, &dev_config, &pressure_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add pressure I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(pressure_bus_handle);
        pressure_bus_handle = NULL;
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief I2C üzerinden sensörden veriyi alır ve belirli sayıda örnek ortalamasını hesaplar.
 *
 * @param i2c_num I2C portu (örneğin, PRESSURE_I2C_NUM veya FLOW_I2C_NUM)
 * @param times   Alınacak örnek sayısı
 * @return int    Ortalama raw ölçüm değeri (hata durumunda -1 döner)
 */
int measure_i2c(i2c_port_t i2c_num, uint8_t times)
{
    int retval = 0;
    uint8_t out[4];
    esp_err_t ret;
    i2c_master_dev_handle_t dev_handle;
    const int max_retries = 3;
    int retry_count = 0;

    // Select the correct device handle based on I2C port
    if (i2c_num == FLOW_I2C_NUM)
    {
        dev_handle = flow_dev_handle;
    }
    else
    {
        dev_handle = pressure_dev_handle;
    }

    for (int i = 0; i < times; i++)
    {
        retry_count = 0;
        while (retry_count < max_retries)
        {
            ret = i2c_master_receive(dev_handle, out, sizeof(out), 1000 / portTICK_PERIOD_MS);
            if (ret == ESP_OK)
            {
                break; // Success, exit retry loop
            }

            // Handle I2C NACK error silently
            if (ret == ESP_ERR_TIMEOUT || ret == ESP_ERR_INVALID_STATE)
            {
                retry_count++;
                vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
                continue;
            }

            // For other errors, return error without logging
            return -1;
        }

        if (retry_count >= max_retries)
        {
            continue; // Skip this measurement but continue with others
        }

        // Ölçüm: İlk 6 bitten maske uygulayarak, 14-bitlik ölçüm değeri elde ediyoruz
        int measurement = ((out[0] & 0x3F) << 8) | out[1];
        retval += measurement;
    }

    if (retval == 0)
    {
        return -1;
    }

    retval = retval / times;
    return retval;
}

//*****************************************************
// Sensör Okuma ve İşleme Task'ı
//*****************************************************

void fp_sensors_task(void *pvParameters)
{
    esp_err_t ret = init_pressure();
    if (ret != ESP_OK)
    {
        // #if FP_SENSORS_DEBUG
        ESP_LOGE(TAG, "Pressure sensor initialization failed");
        // #endif
        esp_restart();
    }
    ret = init_flow();
    if (ret != ESP_OK)
    {
        // #if FP_SENSORS_DEBUG
        ESP_LOGE(TAG, "Flow sensor initialization failed");
        // #endif
        esp_restart();
    }
    ESP_LOGI("FP_SENSORS", "Sensors initialized");
    int final_pressure = 0, final_flow = 0;
    int raw_pressure = 0, raw_flow = 0;

    pressure_base = measure_i2c(PRESSURE_I2C_NUM, 50);
    flow_base = measure_i2c(FLOW_I2C_NUM, 50);

    if (pressure_base < 8250 && pressure_base > 8100 && flow_base < 8250 && flow_base > 8100)
    {
        ESP_LOGI(TAG, "Pressure base: %d , Flow base: %d ", pressure_base, flow_base);
        // #if FP_SENSORS_DEBUG
        // #endif
    }
    else
    {
        ESP_LOGE(TAG, "Pressure base: %d , Flow base: %d ", pressure_base, flow_base);
        esp_restart();
    }
    while (1)
    {

        raw_pressure = measure_i2c(PRESSURE_I2C_NUM, REPEAT_CALCULATION);
        raw_flow = measure_i2c(FLOW_I2C_NUM, REPEAT_CALCULATION);
#if FP_SENSORS_DEBUG
        ESP_LOGI(TAG, "Raw Pressure: %d , Raw Flow: %d ", raw_pressure, raw_flow);
#endif

        if (raw_pressure >= 0 && raw_flow >= 0)
        {
            process_sensor_data(raw_pressure, raw_flow,
                                &final_pressure, &final_flow);

            device_data.pressure = final_pressure - 15;
            device_data.flow = final_flow / 10;

#if FP_SENSORS_DEBUG
            ESP_LOGI(TAG, "Pressure: %d cmH2O, Flow: %d L/min", device_data.pressure, device_data.flow);
#endif
        }

        u_shift_and_insert_int64(device_data.pressure_base_array, MAX_PRESSURE_BASE_COUNT, device_data.pressure);
        u_shift_and_insert_int64(device_data.flow_base_array, MAX_FLOW_BASE_COUNT, device_data.flow);
        device_data.pressure_base = u_average_int64(device_data.pressure_base_array, MAX_PRESSURE_BASE_COUNT);
        device_data.flow_base = u_average_int64(device_data.flow_base_array, MAX_FLOW_BASE_COUNT);

        u_shift_and_insert_int16(device_data.average_flow_array, MAX_AVERAGE_FLOW_COUNT, device_data.flow);
        u_shift_and_insert_int16(device_data.average_pressure_array, MAX_AVERAGE_PRESSURE_COUNT, device_data.pressure);
        device_data.average_flow = u_average_int16(device_data.average_flow_array, MAX_AVERAGE_FLOW_COUNT);
        device_data.average_pressure = u_average_int16(device_data.average_pressure_array, MAX_AVERAGE_PRESSURE_COUNT);

        if (device_data.breath_direction == 0)
        {
            u_shift_and_insert_int16(device_data.flw_array, 25, device_data.flow);
            device_data.average_flw = u_average_int16(device_data.flw_array, 25);
        }
        else
        {
            memset(device_data.flw_array, 0, sizeof(device_data.flw_array));
        }

        vTaskDelay(pdMS_TO_TICKS(4));
#if FP_SENSORS_DEBUG
        ESP_LOGI(TAG, "Pressure base: %d , Flow base: %d ", device_data.pressure_base, device_data.flow_base);
#endif
    }
}