# SPI-SPY

This is a listener and visualizer for reverse-engineering SPI protocol between main CPU and control panel of Pioneer CDJ-1000mk3.

You can see a demonstration [here](https://www.youtube.com/watch?v=yRItR1R8qdQ):

[![View demo on Youtube](https://img.youtube.com/vi/yRItR1R8qdQ/0.jpg)](https://www.youtube.com/watch?v=yRItR1R8qdQ)

## Build and run
This piece of software must be built with [Keil MKD5](http://www2.keil.com/mdk5/) and run on [STM32F7-discovery](http://www.st.com/en/evaluation-tools/32f746gdiscovery.html)

## Connection to main board

![rw](https://github.com/DjFix/spi-spy/blob/master/Doc/Schematics.png)


## Protocol *Main Assy => Display Board*

**0 byte** – number of package

**CRC** - Packet CRC. Calculated as (1byte + 2byte +…+25byte)%256

**WFM** - Waveform bar bitmask

**MB** - Memory bar bitmask

**TB** - Time bar bitmask

**CB** - Cue bar bitmask

**POS** - Jog wheel display register


| / | P0  | P1  | P2  | P3  |  P4  |  P5 |  P6 | P7  |
|---|-----|-----|-----|-----|------|-----|-----|-----|
| 0 | 0   | 24  | 48  | 72  | 96   | 120 | 144 | 168 |
| 1 | WFM | WFM | WFM | WFM | WFM  | TB  |     |     |
| 2 | WFM | WFM | WFM | WFM | WFM  | TB  |     |     |
| 3 | WFM | WFM | WFM | WFM | WFM  | TB  |     |     |
| 4 | WFM | WFM | WFM | WFM | WFM  | TB  |     |     |
| 5 | WFM | WFM | WFM | WFM | MB   | TB  |     |     |
| 6 | WFM | WFM | WFM | WFM | MB   | TB  |     |     |
| 7 | WFM | WFM | WFM | WFM | MB   | TB  |     |     |
| 8 | WFM | WFM | WFM | WFM | MB   | TB  |     |     |
| 9 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|10 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|11 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|12 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|13 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|14 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|15 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|16 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|17 | WFM | WFM | WFM | WFM | MB   | CB  |     |     |
|18 | WFM | WFM | WFM | WFM | 0x00 | CB  |     |     |
|19 | WFM | WFM | WFM | WFM | TB   | CB  |     |     |
|20 | WFM | WFM | WFM | WFM | TB   | CB  |     |     |
|21 | WFM | WFM | WFM | WFM | TB   | CB  |     |     |
|22 | WFM | WFM | WFM | WFM | TB   | 0x00|     |     |
|23 | WFM | WFM | WFM | WFM | TB   | 0x00|     |     |
|24 | WFM | WFM | WFM | WFM | TB   | 0x00|     |     |
|25 | POS | POS | POS | POS | POS  | POS | POS | POS |
|26 | CRC | CRC | CRC | CRC | CRC  | CRC | CRC | CRC |

Description of P6-P7 TBD. See [original doc](https://github.com/DjFix/spi-spy/blob/master/Doc/Reverse_Engineering_Pioneer_CDJ-1000_serial_protocol.pdf) for more info

## Protocol *Display Board => Main Assy*

**ATB** -  ADC data TOUCH/BRAKE 0…255 8 bits resolution

**ARS** - ADC data RELEASE/START 0…255 8 bits resolution

**APH** -  ADC Pitch MSB [AAAAAAAA] 0…16383 14 bits resolution

**APL** - ADC Pitch LSB [AAAAAA00]

**APCH** -  ADC Pitch Center Potentiometer MSB [AAAAAAAA] 0…16383 14 bits resolution

**APCL** - ADC Pitch Center Potentiometer LSB [AAAAAA00]

**JPH** -  Jog Pulse Counter MSB 0…65535 16 bits resolution (3600 pulses per 1 round)

**JPL** -  Jog Pulse Counter LSB

**JSH** -  Jog Speed MSB 0…65535 16 bits resolution (when jog stopped speed = 65535)

**JSL** - Jog Speed LSB

**JST** - Jog status

**B0 - B3** - Buttons bitmask

| / | P0-7 |
|---|------|
| 0 | 1    |
| 1 | 16   |
| 2 | ATB  |
| 3 | ARS  |
| 4 | APH  |
| 5 | APL  |
| 6 | APCH |
| 7 | APCL |
| 8 | JPH  |
| 9 | JPL  |
|10 | JSH  |
|11 | JSL  |
|12 | JST  |
|13 | 0x00 |
|14 | B0   |
|15 | 0x00 |
|16 | B1   |
|17 | B2   |
|18 | B3   |
|19 | CRC? |
|20 | 241  |
|21 | 100  |
|22 | 118  |
|23 | 0x00 |
|24 | 0x00 |
|25 | 0x00 |
|26 | 0x00 |

B0-B3 description TBD. See [original doc](https://github.com/DjFix/spi-spy/blob/master/Doc/Reverse_Engineering_Pioneer_CDJ-1000_serial_protocol.pdf) for more info

-------
Author of research: [Anatsko Andrei](https://github.com/djgreeb)
