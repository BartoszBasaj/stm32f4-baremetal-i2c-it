# STM32F4 Bare-Metal Interrupt-Driven I2C Driver (with BME280 & IWDG)

Non-blocking (interrupt-driven) I2C1 master driver written from scratch in C using direct CMSIS register access for the **STM32F446RE** (ARM Cortex-M4), with zero HAL/LL dependencies.

The project is structured into three layers: a generic hardware I2C driver (`driver/`), a modular **Bosch BME280** sensor driver (`device/`) with 32-bit fixed-point compensation, and an application layer (`app/`) featuring **Independent Watchdog (IWDG)** fault recovery and real-time SWV ITM logging. It also includes a zero-dependency interactive HTML/SVG waveform and FSM simulator (`index.html`).

![Demo GIF](docs/gif.gif)

---

## Solved STM32F4 Hardware Quirks (RM0390)

The STM32F4 I2C peripheral requires strict hardware event sequencing during multi-byte reception so that `NACK` and `STOP` bits are set before the final byte finishes shifting in. This driver uses a deterministic state machine inside `I2C1_EV_IRQHandler` and `I2C1_ER_IRQHandler` to handle all edge cases:

* **Multi-Byte Reception Sequencing (1, 2, and N >= 3 Bytes):**
  * **N = 1:** Clears `ACK` while `SCL` is stretched low during the `ADDR` event, clears `ADDR` (`SR1` -> `SR2` read), and programs `STOP` immediately before the byte starts shifting in.
  * **N = 2:** Clears `ACK`, sets the hardware `POS` bit (`I2C_CR1_POS`) during `ADDR`, and keeps buffer interrupts (`ITBUFEN`) disabled. The hardware automatically ACKs the 1st byte and NACKs the 2nd byte. Once both the Data Register (`DR`) and Shift Register are full (`BTF = 1`, `SCL` stretched low), the ISR issues `STOP` and reads `DR` twice back-to-back.
  * **N >= 3:** Reads incoming bytes via `RxNE` (`ITBUFEN = 1`) until `rx_count == 3`, then dynamically disables `ITBUFEN` to hand control over to `BTF` double-buffering. When byte `N-2` is in `DR` and `N-1` is in the Shift Register (`BTF = 1`), the ISR clears `ACK` and reads `N-2`. On the final `BTF` (`N-1` in `DR`, `N` in Shift Register), it asserts `STOP` and reads the last two bytes back-to-back.
* **Repeated START Race Condition Fix:**
  * Setting `CR1.START` immediately after sending the register address can cause the CPU to exit and re-enter the `BTF` interrupt before the hardware finishes asserting the Repeated START condition on the bus, shifting the readout buffer by 1 byte.
  * Fixed by performing a dummy read `(void)I2C1->DR` to clear `BTF` immediately and transitioning through a dedicated `I2C_STATE_SEND_RADDR` state that waits strictly for `SB = 1`.
* **Bus Lockup & Fault Recovery (`I2C1_ER_IRQHandler` + `IWDG`):**
  * Clears hardware error flags (`BERR`, `ARLO`, `AF`, `OVR`), issues a `STOP` condition to release the bus, and unblocks the application loop via `i2c_error`.
  * Executes a software peripheral reset (`SWRST`) in `I2C_Init()` to clear stuck internal silicon states after mid-transfer MCU resets.
  * Uses the hardware **Independent Watchdog (IWDG)** clocked by the independent 32 kHz LSI oscillator (2-second timeout) to trigger a full MCU reset if a physical line disconnect or persistent bus fault occurs.

---

## Finite State Machine (FSM)

```text
[IDLE]
  │ (I2C1_Read_IT / I2C1_Write_IT -> CR1.START)
  ▼
[SEND_ADDR] ──(SB=1)──► DR = (addr << 1) | 0 (Write)
  │
  ▼
[SEND_REG]  ──(ADDR=1)► Clear ADDR, DR = reg_addr
  │
  ├──► (Write) ──(TXE/BTF)──► [SEND_DATA] ──(tx_count=0)──► [STOP_WAIT] ──(BTF=1)──► CR1.STOP & [IDLE]
  │
  └──► (Read)  ──(BTF=1)────► [RESTART] (Dummy DR read + CR1.START)
                                 │
                                 ▼
                            [SEND_RADDR] ──(SB=1)──► DR = (addr << 1) | 1 (Read)
                                 │
                                 ▼
                            [READ_DATA]  ──(ADDR=1)► Configure ACK/POS/ITBUFEN, clear ADDR
                                 │
                                 └──(RxNE / BTF)──► Read N Bytes ──► CR1.STOP & [IDLE]
```

---

## Hardware Setup

| Signal | STM32F446RE Pin | Peripheral Config | BME280 Pin |
| :--- | :--- | :--- | :--- |
| **I2C1_SCL** | PB8 | AF4, Open-Drain, 100 kHz | SCL |
| **I2C1_SDA** | PB9 | AF4, Open-Drain, 100 kHz | SDA |
| **SWO (Trace)** | PB3 | SWV ITM Port 0 (`printf`) | — |
| **Power** | 3.3V / GND | — | VCC / GND |

*Note: BME280 I2C address is `0x76` (`SDO` connected to `GND`).*

---

## Repository Structure

```text
├── driver/
│   ├── i2c.h             # Hardware Layer: Public I2C1 API and status flags
│   └── i2c.c             # Hardware Layer: Bare-metal I2C1 state machine & ISRs
├── device/
│   ├── bme280.h          # Device Layer: BME280 API, status codes & data struct
│   └── bme280.c          # Device Layer: Sensor init, burst read & Bosch fixed-point math
├── app/
│   └── main.c            # Application Layer: Main loop, IWDG watchdog & SWV logging
├── docs/
│   └── gif.gif           # Live debugging & fault recovery demo
├── index.html            # Interactive step-by-step I2C waveform & FSM simulator
└── README.md
```

---

## SWV ITM Console Output (Fault Recovery Test)

Real-time trace captured via ST-Link SWO (`ITM_SendChar`) showing normal readout, a simulated physical `SDA` wire disconnect, and automatic recovery via `IWDG`:

```text
[INFO] Initializing I2C1 (IT) and IWDG...
[INFO] BME280 initialized (Chip ID: 0x60). Calibration loaded.
[DATA] TEMP: 23.48 C | PRESS: 991.15 hPa
[DATA] TEMP: 23.49 C | PRESS: 991.14 hPa
[ERR] I2C bus fault! Waiting for IWDG hardware reset...
[ERR] I2C bus fault! Waiting for IWDG hardware reset...
[WARN] System recovered from IWDG reset (I2C bus lockup detected)!
[INFO] Initializing I2C1 (IT) and IWDG...
[INFO] BME280 initialized (Chip ID: 0x60). Calibration loaded.
[DATA] TEMP: 23.48 C | PRESS: 991.15 hPa
```

---

## Prerequisites & How to Build

This repository contains only the clean source files and requires standard **ARM CMSIS** device headers (`#include "stm32f4xx.h"`):

1. **Create an Empty Project in STM32CubeIDE:**
   * Go to **File -> New -> STM32 Project**, select `STM32F446RETx`, and choose **Empty** as the Targeted Project Type (ensuring CMSIS headers and the `STM32F446xx` preprocessor define are included).
2. **Add Source Files:**
   * Copy `driver/i2c.c`, `device/bme280.c`, and `app/main.c` into your project's `Src/` folder, and `driver/i2c.h`, `device/bme280.h` into `Inc/`.
3. **Configure SWV Trace:**
   * In **Debug Configurations -> Debugger**, enable **Serial Wire Viewer (SWV)** and set **Core Clock** to `16.0 MHz` (default HSI).
   * Start debugging, open **SWV ITM Data Console**, enable **Port 0**, and click **Start Trace**.
4. **Interactive Waveform & FSM Visualizer:**
   * Open `visualizer.html` in any browser to step through the `SCL`/`SDA` waveforms, clock stretching zones, `SR1`/`DR`/Shift Register states, and the interactive FSM graph.