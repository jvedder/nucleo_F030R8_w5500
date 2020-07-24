# WizNet W5500 driver for STM32F030R8

Tinkering with WinNet W5500 Ethernet Chip using an STM32F030R8 Nucleo Board

## Requirements

 - "Nucleo-F030R9" Developement Board from [ST Microelectronics](https://www.st.com/)
 - Adafruit "Ethernet FeatherWing" [Item #3201](https://www.adafruit.com/product/3201)
 - Eclipse CDT with "System Workbench for STM32" from [openstm32.org](https://www.openstm32.org/)
 - Serial Port Terminal (PuTTY, TeraTerm or equivalent). 

## Setup

 - Connect FeatherWing GND to Nucleo CN7 Pin 8 (GND)
 - Connect FeatherWing 3V to Nucleo CN7 Pin 12 (+3V3)
 - Connect FeatherWing MOSI to Nucleo CN10 Pin 26 (PB15)
 - Connect FeatherWing MISO to Nucleo CN10 Pin 28 (PB14)
 - Connect FeatherWing SCK to Nucleo CN10 Pin 26 (PB13)
 - Connect FeatherWing CS to Nucleo CN10 Pin 16 (PB12)
 - ST-LINK serial port is set to 38400, 8N1
 
   
## Usage

 - TBD

## License:

[MIT License](../master/LICENSE.txt)
