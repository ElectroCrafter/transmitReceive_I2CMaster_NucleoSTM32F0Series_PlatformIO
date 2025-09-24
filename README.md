# transmitReceive_I2CMaster_NucleoSTM32F0Series_PlatformIO

Bare-metal demo for STM32F072RB (Nucleo board) implementing I²C master transmit & receive.

Features
- TIM2-based millis emulation (no interrupts used for timing)
- USART2 debug prints (PA2/PA3, 9600 baud)
- I2C1 master on PB6 (SCL) / PB7 (SDA)
- Periodically reads a small message from a slave (address 0x08)
- Example code demonstrates both transmit and receive helpers

Hardware
- Board: Nucleo-F072RB
- I2C: PB6 (SCL), PB7 (SDA) with pull-ups (4.7k recommended)
- Slave example address: 0x08 (Arduino or other device)

Build
```bash
pio run
