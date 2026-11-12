/*
 * bme280.c
 *
 *  Created on: Nov 12, 2026
 *      Author: bhbas
 */

#include "bme280.h"
#include "i2c.h"

// Internal sensor buffers hidden from main.c
static uint8_t calib_data[24] = {0};
static uint8_t raw_measurement[6] = {0};
static int32_t t_fine = 0;

// Bosch 32-bit fixed-point temperature compensation (returns 0.01 DegC)
static int32_t bme280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2;

    uint16_t dig_T1 = ((uint16_t)calib_data[1] << 8) | calib_data[0];
    int16_t  dig_T2 = (int16_t)(((uint16_t)calib_data[3] << 8) | calib_data[2]);
    int16_t  dig_T3 = (int16_t)(((uint16_t)calib_data[5] << 8) | calib_data[4]);

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

// Bosch 32-bit fixed-point pressure compensation (returns Pa)
static uint32_t bme280_compensate_P_int32(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;

    uint16_t dig_P1 = ((uint16_t)calib_data[7] << 8) | calib_data[6];
    int16_t  dig_P2 = (int16_t)(((uint16_t)calib_data[9] << 8) | calib_data[8]);
    int16_t  dig_P3 = (int16_t)(((uint16_t)calib_data[11] << 8) | calib_data[10]);
    int16_t  dig_P4 = (int16_t)(((uint16_t)calib_data[13] << 8) | calib_data[12]);
    int16_t  dig_P5 = (int16_t)(((uint16_t)calib_data[15] << 8) | calib_data[14]);
    int16_t  dig_P6 = (int16_t)(((uint16_t)calib_data[17] << 8) | calib_data[16]);
    int16_t  dig_P7 = (int16_t)(((uint16_t)calib_data[19] << 8) | calib_data[18]);
    int16_t  dig_P8 = (int16_t)(((uint16_t)calib_data[21] << 8) | calib_data[20]);
    int16_t  dig_P9 = (int16_t)(((uint16_t)calib_data[23] << 8) | calib_data[22]);

    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = (((32768 + var1)) * ((int32_t)dig_P1)) >> 15;

    if (var1 == 0) return 0;

    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

BME280_Status_t BME280_Init(uint8_t *detected_chip_id) {
    uint8_t hum_config = 0x01; // Humidity oversampling x1
    uint8_t config_val = 0x27; // Pressure & Temp oversampling x1, Normal mode
    uint8_t chip_id = 0;

    // Allow NVM calibration copy to complete after power-on
    for (volatile uint32_t i = 0; i < 100000; i++);

    // 1. Configure humidity oversampling (must be written before ctrl_meas)
    I2C1_Write_IT(BME280_I2C_ADDR, BME280_REG_CTRL_HUM, &hum_config, 1);
    while (data_sent == 0 && i2c_error == 0) {}
    data_sent = 0;
    if (i2c_error) return BME280_ERR_I2C;

    // 2. Configure temperature/pressure oversampling and Normal power mode
    I2C1_Write_IT(BME280_I2C_ADDR, BME280_REG_CTRL_MEAS, &config_val, 1);
    while (data_sent == 0 && i2c_error == 0) {}
    data_sent = 0;
    if (i2c_error) return BME280_ERR_I2C;

    // 3. Read and verify Chip ID (0x60 = BME280, 0x58 = BMP280)
    I2C1_Read_IT(BME280_I2C_ADDR, BME280_REG_CHIP_ID, &chip_id, 1);
    while (data_ready == 0 && i2c_error == 0) {}
    data_ready = 0;
    if (detected_chip_id) *detected_chip_id = chip_id;
    if (i2c_error) return BME280_ERR_I2C;
    if (chip_id != 0x60 && chip_id != 0x58) return BME280_ERR_ID;

    // 4. Burst-read 24 factory calibration bytes (0x88..0x9F)
    I2C1_Read_IT(BME280_I2C_ADDR, BME280_REG_CALIB, calib_data, 24);
    while (data_ready == 0 && i2c_error == 0) {}
    data_ready = 0;
    if (i2c_error) return BME280_ERR_I2C;

    return BME280_OK;
}

void BME280_StartMeasurement_IT(void) {
    // Non-blocking trigger: burst-read 6 bytes (0xF7..0xFC) in the background
    I2C1_Read_IT(BME280_I2C_ADDR, BME280_REG_DATA, raw_measurement, 6);
}

BME280_Status_t BME280_ProcessMeasurement(BME280_Data_t *out) {
    if (i2c_error) {
        return BME280_ERR_I2C;
    }
    if (data_ready == 0) {
        return BME280_BUSY; // Transfer still in progress
    }

    data_ready = 0;

    // Assemble 20-bit unsigned ADC values
    int32_t adc_P = ((int32_t)raw_measurement[0] << 12) | ((int32_t)raw_measurement[1] << 4) | (raw_measurement[2] >> 4);
    int32_t adc_T = ((int32_t)raw_measurement[3] << 12) | ((int32_t)raw_measurement[4] << 4) | (raw_measurement[5] >> 4);

    out->temp_c_x100 = bme280_compensate_T_int32(adc_T);
    out->press_pa    = bme280_compensate_P_int32(adc_P);

    out->temperature_c = (float)out->temp_c_x100 / 100.0f;
    out->pressure_hpa  = (float)out->press_pa / 100.0f;

    return BME280_OK;
}