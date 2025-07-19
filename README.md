# MS1 logger for Achipelag

## Hardware

ESP32-CAM. PSRAM removed.
Pin "VCC" on header P1 is disconnected from 5/3.3V and used as GPIO_32 (disconnected from CAM_PWR).

### Pin assignments

LED is GPIO_33

SD card pin | ESP32 pin 
------------|--------------     
D0  (MISO)  | GPIO_2
D3  (CS)    | GPIO_13
CLK (SCK)   | GPIO_14
CMD (MOSI)  | GPIO_15

MS UART pin | ESP32 pin 
------------|--------------     
RX          | GPIO_16
TX          | GPIO_0

## Flashing

Take the ESP board out, short IO0 to GND. This enabled download boot mode.
Use FT2232 or similar modules as USB-UART converters. For FT2232HL AD0<->U0R, AD1<->U0T.
