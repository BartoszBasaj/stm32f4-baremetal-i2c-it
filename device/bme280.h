/*
 * bme280.h
 *
 *  Created on: Nov 12, 2026
 *      Author: bhbas
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>

#define BME280_I2C_ADDR     0x76

#define BME280_REG_CALIB    0x88
#define BME280_REG_CHIP_ID  0xD0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_DATA     0xF7

typedef enum {
    BME280_OK = 0,
    BME280_BUSY = 1,
    BME280_ERR_I2C = -1,
    BME280_ERR_ID = -2
} BME280_Status_t;

typedef struct {
    int32_t temp_c_x100;  // e.g. 2348 = 23.48 C
    uint32_t press_pa;    // e.g. 99115 = 991.15 hPa
    float temperature_c;
    float pressure_hpa;
} BME280_Data_t;

// Public Device API
BME280_Status_t BME280_Init(uint8_t *detected_chip_id);
void BME280_StartMeasurement_IT(void);
BME280_Status_t BME280_ProcessMeasurement(BME280_Data_t *out);

#endif /* BME280_H_ */