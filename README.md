
# ESP32 Oscilloscope Test Board Pinout & Signal Description

| **Pin (GPIO)** | **Function**   | **Direction** | **Details**                                                  |
| :------------: | :------------- | :-----------: | :----------------------------------------------------------- |
|       21       | LED1 / I2C SDA |    Output     | LED1 toggled at 1000ms interval, I2C SDA for OLED (0x3C)     |
|       22       | LED2 / I2C SCL |    Output     | LED2 toggled at 330ms interval, I2C SCL for OLED (0x3C)      |
|       23       | LED3           |    Output     | LED3 toggled at 50ms interval                                |
|       32       | PWM1           |    Output     | PWM, 1kHz, 25% duty cycle                                    |
|       33       | PWM2           |    Output     | PWM, 1kHz, 50% duty cycle                                    |
|       25       | PWM3           |    Output     | PWM, 1kHz, 75% duty cycle                                    |
|       13       | Button         |     Input     | Button input, active low, measures press duration            |
|       5        | UART1 TX       |    Output     | Serial output, **115200 baud**, sends button press duration  |
|       18       | UART2 TX       |    Output     | Serial output, **9600 baud**, sends "Hello, World!" every 2s |
|       21       | I2C SDA (OLED) |    Output     | Shared with LED1, OLED SSD1306, address 0x3C                 |
|       22       | I2C SCL (OLED) |    Output     | Shared with LED2, OLED SSD1306, address 0x3C                 |

---

## **Baud Rate Settings**

- **UART1 (TX on GPIO 5):** 115200 baud  
  _Sends button press duration as text (e.g., "123 ms\r\n")_
- **UART2 (TX on GPIO 18):** 9600 baud  
  _Sends "Hello, World!" every 2 seconds_

---

## **For Oscilloscope/Logic Analyzer Study**

- **LED Pins (21, 22, 23):** Observe digital toggling at different intervals.
- **PWM Pins (32, 33, 25):** Observe PWM waveforms at 1kHz with 25%, 50%, and 75% duty cycles.
- **UART Pins (5, 18):** Capture serial data at specified baud rates.
- **Button Pin (13):** Use as a digital input, observe debounce and timing.
- **I2C Pins (21, 22):** Observe I2C communication with OLED (address 0x3C).

---

## **Note for Students**

- Use a logic analyzer to capture UART, PWM, and I2C signals.
- Use an oscilloscope to measure PWM duty cycles and LED blink intervals.
- Ensure your serial terminal matches the correct baud rate for each UART output.

---

