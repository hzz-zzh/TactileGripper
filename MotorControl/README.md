# H723 tactile gripper motor control

This directory contains the STM32H723 board adaptation of MCSDK 6.4.1.
The generic MCSDK library sources are referenced from `Reference/H745_MCSDK_OS` by the Keil project; generated board configuration and application code live here.

## Hardware mapping

| Function | MCU resource |
| --- | --- |
| U/V/W high-side PWM | PE9 / PE11 / PE13, TIM1 CH1/2/3 |
| U/V/W low-side PWM | PE8 / PE10 / PE12, TIM1 CH1N/2N/3N |
| Phase current A/B/C | PA1 ADC1_IN17 / PA2 ADC2_IN14 / PA5 ADC2_IN19 |
| Bus current | PA6 ADC2_IN3 |
| Bus voltage | PA7 ADC2_IN7 |
| External NTC input | PA3 ADC1_IN15 |
| KTH7812 | SPI1: PB3 SCK, PB4 MISO, PB5 MOSI, PA15 hardware NSS |
| Motor Pilot | USART1: PA9 TX, PA10 RX, 1843200 baud |
| Debug console | USART2: PD5 TX, PD6 RX, 115200 baud |

The SPI1 mapping above is the PCB source of truth and intentionally supersedes the older schematic net labels.

## Safety defaults

- 16 kHz center-aligned PWM, 500 ns timer dead time; FD6288 adds its internal dead time.
- 24 V nominal bus, 10 V undervoltage and 30 V overvoltage limits.
- 1 A software phase-current limit and 2 A bus-current analog-watchdog trip.
- Automatic alignment and homing use a 0.3 A speed-loop output limit.
- Any MCSDK, encoder CRC/SPI, homing timeout, travel, voltage, or current fault switches PWM off and latches a fault.

Do the first power test with a current-limited 24 V supply and the motor mechanically unloaded. Verify all six gate signals and ADC bias before fitting the gripper.

PA3 is sampled for the external NTC connector, but over-temperature shutdown is intentionally not enabled until the fitted NTC's R25/Beta curve is known and calibrated. The schematic specifies only the board-side 10 kOhm pull-up; applying the imported reference-board temperature curve would create a false sense of protection.

## Startup sequence

After the RTOS starts, the service checks the encoder and bus voltage, performs a 0.3 A electrical alignment, finds the opening stop, finds the closing stop, then returns to 5% closed. Position commands use 0 for fully open and 1000 for fully closed.

The public interface is declared in `gripper_motor_service.h`. Motor Pilot uses the standard MCSDK ASPEP interface over UART1.

## Build and regeneration

Build `MDK-ARM/TactileGripper_H723VGH6.uvprojx` with Keil ARMCC 5.06. The current `.ioc` predates this manual H723 MCSDK port; regenerating code from CubeMX would overwrite the hand-adapted peripheral and interrupt setup. Update the `.ioc` first if CubeMX regeneration becomes necessary.
