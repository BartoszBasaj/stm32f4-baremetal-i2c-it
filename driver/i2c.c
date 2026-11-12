/*
 * i2c.c
 *
 *  Created on: Nov 11, 2026
 *      Author: bhbas
 */

#include "i2c.h"
#include "stm32f4xx.h"

#define I2C_ITERREN (1U << 8)
#define I2C_ITEVTEN (1U << 9)
#define I2C_ITBUFEN (1U << 10)

typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_SEND_ADDR,    // START sent, waiting for SB to transmit Slave Write address
    I2C_STATE_SEND_REG,     // Slave ACKed, transmitting internal register address
    I2C_STATE_RESTART,      // Register sent, waiting for BTF to issue Repeated START
    I2C_STATE_SEND_RADDR,   // Repeated START sent, waiting for SB to transmit Slave Read address
    I2C_STATE_READ_DATA,    // Receiving payload bytes from Slave
    I2C_STATE_SEND_DATA,    // Transmitting payload bytes to Slave
    I2C_STATE_STOP_WAIT     // Payload sent, waiting for BTF before issuing STOP
} I2C_State;

// Private driver context isolated via 'static'
static volatile I2C_State i2c_state = I2C_STATE_IDLE;
static volatile uint8_t device_addr;
static volatile uint8_t register_addr;

static uint8_t * volatile rx_buffer;
static volatile uint8_t rx_count;

static uint8_t * volatile tx_buffer;
static volatile uint8_t tx_count;

static volatile uint8_t is_read_operation = 0;

// Public status flags
volatile uint8_t data_ready = 0;
volatile uint8_t data_sent = 0;
volatile uint8_t i2c_error = 0;

void I2C_Init(void) {
    RCC->AHB1ENR |= (1U << 1);  // Enable GPIOB clock
    RCC->APB1ENR |= (1U << 21); // Enable I2C1 clock

    // Configure PB8 (SCL) & PB9 (SDA): AF4, Open-Drain
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  ((2U << 16) | (2U << 18));
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB->AFR[1] |=  ((4U << 0) | (4U << 4));

    // Software reset to clear internal hardware state machine in case of mid-transfer MCU reset
    I2C1->CR1 |= (1U << 15);
    I2C1->CR1 &= ~(1U << 15);

    // Timing for 100 kHz Standard Mode (16 MHz APB1)
    I2C1->CR2 = 16U;
    I2C1->CCR = 80U;
    I2C1->TRISE = 17U;

    // Enable Event and Error interrupts (ITBUFEN is toggled dynamically)
    I2C1->CR2 |= (I2C_ITEVTEN | I2C_ITERREN);
    I2C1->CR1 |= (1U << 0);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void I2C1_Read_IT(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint32_t timeout = 100000;

    if (i2c_state != I2C_STATE_IDLE || len == 0 || buf == 0) return;

    // Wait for hardware to clear previous STOP condition
    while (I2C1->CR1 & (1U << 9)) {
        if (--timeout == 0) {
            i2c_error = 1;
            data_ready = 1;
            return;
        }
    }

    I2C1->CR1 &= ~(1U << 11); // Clear POS bit
    I2C1->CR2 &= ~I2C_ITBUFEN;

    is_read_operation = 1;
    i2c_error = 0;
    data_ready = 0;

    device_addr = dev;
    register_addr = reg;
    rx_buffer = buf;
    rx_count = len;

    i2c_state = I2C_STATE_SEND_ADDR;
    I2C1->CR1 |= (1U << 8); // Generate START
}

void I2C1_Write_IT(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint32_t timeout = 100000;

    if (i2c_state != I2C_STATE_IDLE || len == 0 || buf == 0) return;

    while (I2C1->CR1 & (1U << 9)) {
        if (--timeout == 0) {
            i2c_error = 1;
            data_sent = 1;
            return;
        }
    }

    I2C1->CR1 &= ~(1U << 11);
    I2C1->CR2 &= ~I2C_ITBUFEN;

    is_read_operation = 0;
    i2c_error = 0;
    data_sent = 0;

    device_addr = dev;
    register_addr = reg;
    tx_buffer = buf;
    tx_count = len;

    i2c_state = I2C_STATE_SEND_ADDR;
    I2C1->CR1 |= (1U << 8);
}

void I2C1_EV_IRQHandler(void) {
    uint32_t sr1 = I2C1->SR1;
    volatile uint32_t tmp;

    // 1. Start Bit (SB = 1)
    if (sr1 & (1U << 0)) {
        if (i2c_state == I2C_STATE_SEND_ADDR) {
            I2C1->DR = (device_addr << 1); // Write mode (R/W = 0)
            i2c_state = I2C_STATE_SEND_REG;
        }
        else if (i2c_state == I2C_STATE_SEND_RADDR) {
            I2C1->DR = (device_addr << 1) | 1U; // Read mode (R/W = 1)
            i2c_state = I2C_STATE_READ_DATA;
        }
    }

    // 2. Address Sent (ADDR = 1): SCL stretched low until SR1->SR2 read sequence
    else if (sr1 & (1U << 1)) {
        if (i2c_state == I2C_STATE_SEND_REG) {
            tmp = I2C1->SR1;
            tmp = I2C1->SR2;
            (void)tmp;

            I2C1->DR = register_addr;

            if (is_read_operation) {
                i2c_state = I2C_STATE_RESTART;
            } else {
                i2c_state = I2C_STATE_SEND_DATA;
                I2C1->CR2 |= I2C_ITBUFEN;
            }
        }
        else if (i2c_state == I2C_STATE_READ_DATA) {
            if (rx_count == 1) {
                I2C1->CR1 &= ~(1U << 10); // NACK immediate byte
            } else if (rx_count == 2) {
                I2C1->CR1 &= ~(1U << 10); // Clear ACK
                I2C1->CR1 |= (1U << 11);  // POS = 1: NACK applies to 2nd byte
            } else {
                I2C1->CR1 |= (1U << 10);  // ACK incoming bytes
            }

            // Clear ADDR to release SCL
            tmp = I2C1->SR1;
            tmp = I2C1->SR2;
            (void)tmp;

            if (rx_count == 1) {
                I2C1->CR1 |= (1U << 9);   // Program STOP immediately
                I2C1->CR2 |= I2C_ITBUFEN;
            } else if (rx_count == 2 || rx_count == 3) {
                // Rely on BTF double-buffering for 2- and 3-byte transfers
                I2C1->CR2 &= ~I2C_ITBUFEN;
            } else {
                I2C1->CR2 |= I2C_ITBUFEN;
            }
        }
    }

    // 3. Byte Transfer Finished (BTF = 1): Both DR and Shift Register are full/empty
    else if (sr1 & (1U << 2)) {
        if (i2c_state == I2C_STATE_RESTART) {
            (void)I2C1->DR; // Dummy read clears BTF immediately before Repeated START
            I2C1->CR1 |= (1U << 8);
            i2c_state = I2C_STATE_SEND_RADDR;
        }
        else if (i2c_state == I2C_STATE_STOP_WAIT) {
            I2C1->CR1 |= (1U << 9);
            i2c_state = I2C_STATE_IDLE;
            data_sent = 1;
        }
        else if (i2c_state == I2C_STATE_READ_DATA) {
            if (rx_count == 3) {
                // Byte N-2 in DR, Byte N-1 in Shift Register (SCL stretched). Set NACK for Byte N.
                I2C1->CR1 &= ~(1U << 10);
                *rx_buffer++ = I2C1->DR;
                rx_count--;
            }
            else if (rx_count == 2) {
                // Byte N-1 in DR, Byte N in Shift Register (SCL stretched). Set STOP and burst-read both.
                I2C1->CR1 |= (1U << 9);
                *rx_buffer++ = I2C1->DR;
                *rx_buffer++ = I2C1->DR;
                rx_count = 0;
                i2c_state = I2C_STATE_IDLE;
                data_ready = 1;
            }
            else if (rx_count > 3) {
                *rx_buffer++ = I2C1->DR;
                rx_count--;
                if (rx_count == 3) {
                    I2C1->CR2 &= ~I2C_ITBUFEN;
                }
            }
        }
        else if (i2c_state == I2C_STATE_SEND_DATA) {
            if (tx_count > 0) {
                I2C1->DR = *tx_buffer++;
                tx_count--;
                if (tx_count == 0) {
                    I2C1->CR2 &= ~I2C_ITBUFEN;
                    i2c_state = I2C_STATE_STOP_WAIT;
                }
            }
        }
    }

    // 4. Receive Buffer Not Empty (RxNE = 1)
    else if ((sr1 & (1U << 6)) && (i2c_state == I2C_STATE_READ_DATA)) {
        if (rx_count > 3) {
            *rx_buffer++ = I2C1->DR;
            rx_count--;
            if (rx_count == 3) {
                I2C1->CR2 &= ~I2C_ITBUFEN;
            }
        }
        else if (rx_count == 1) {
            *rx_buffer++ = I2C1->DR;
            rx_count = 0;
            I2C1->CR2 &= ~I2C_ITBUFEN;
            i2c_state = I2C_STATE_IDLE;
            data_ready = 1;
        }
    }

    // 5. Transmit Buffer Empty (TXE = 1)
    else if ((sr1 & (1U << 7)) && (i2c_state == I2C_STATE_SEND_DATA)) {
        if (tx_count > 0) {
            I2C1->DR = *tx_buffer++;
            tx_count--;
            if (tx_count == 0) {
                I2C1->CR2 &= ~I2C_ITBUFEN;
                i2c_state = I2C_STATE_STOP_WAIT;
            }
        }
    }
}

void I2C1_ER_IRQHandler(void) {
    I2C1->CR1 &= ~(1U << 11);
    I2C1->SR1 &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11)); // Clear BERR, ARLO, AF, OVR
    I2C1->CR1 |= (1U << 9); // Release bus via STOP
    I2C1->CR2 &= ~I2C_ITBUFEN;

    i2c_state = I2C_STATE_IDLE;
    i2c_error = 1;

    if (is_read_operation) {
        data_ready = 1;
    } else {
        data_sent = 1;
    }
}