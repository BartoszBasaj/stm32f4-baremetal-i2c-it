/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Application layer – I2C1, BME280 & IWDG fault recovery
  ******************************************************************************
  */

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "i2c.h"
#include "bme280.h"

// Global variables for STM32CubeIDE Live Expressions inspection
BME280_Data_t env_data;

// Redirect printf to SWV ITM Data Console (Port 0) over SWO pin
int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}

// Configure Independent Watchdog (32 kHz LSI, Prescaler = 32 -> ~2.0s timeout)
static void IWDG_Init(void) {
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; // Freeze IWDG during breakpoint halt

    IWDG->KR = 0x5555; // Unlock PR and RLR registers
    IWDG->PR = 0x03;   // Divide LSI by 32 (1 tick ≈ 1 ms)
    IWDG->RLR = 2000;  // 2000 ms timeout
    IWDG->KR = 0xAAAA; // Reload counter
    IWDG->KR = 0xCCCC; // Enable watchdog
}

static inline void IWDG_Feed(void) {
    IWDG->KR = 0xAAAA;
}

int main(void) {
    uint8_t chip_id = 0;

    // Check if the MCU recovered from an Independent Watchdog reset (IWDGRSTF)
    if (RCC->CSR & (1U << 29)) {
        printf("[WARN] System recovered from IWDG reset (I2C bus lockup detected)!\n");
        RCC->CSR |= (1U << 24); // Clear reset flags (RMVF)
    }

    printf("[INFO] Initializing I2C1 (IT) and IWDG...\n");
    I2C_Init();
    IWDG_Init();

    if (BME280_Init(&chip_id) == BME280_OK) {
        printf("[INFO] BME280 initialized (Chip ID: 0x%02X). Calibration loaded.\n", chip_id);
        IWDG_Feed();
    } else {
        printf("[ERR] BME280 initialization failed (Read ID: 0x%02X)!\n", chip_id);
    }

    for (;;) {
        // Trigger non-blocking 6-byte readout via I2C1 interrupts
        BME280_StartMeasurement_IT();

        // Wait for asynchronous transfer to complete (CPU can execute other tasks here)
        BME280_Status_t status;
        do {
            status = BME280_ProcessMeasurement(&env_data);
        } while (status == BME280_BUSY);

        if (status == BME280_OK) {
            int32_t t_int = env_data.temp_c_x100 / 100;
            int32_t t_frac = (env_data.temp_c_x100 < 0 ? -env_data.temp_c_x100 : env_data.temp_c_x100) % 100;
            uint32_t p_int = env_data.press_pa / 100;
            uint32_t p_frac = env_data.press_pa % 100;

            printf("[DATA] TEMP: %ld.%02ld C | PRESS: %lu.%02lu hPa\n", t_int, t_frac, p_int, p_frac);

            IWDG_Feed();
        } else {
            // Do not feed IWDG on error; persistent bus fault triggers MCU reset after 2s
            printf("[ERR] I2C bus fault! Waiting for IWDG hardware reset...\n");
        }

        for (volatile uint32_t i = 0; i < 2000000; i++);
    }
}