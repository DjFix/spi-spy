# SPI-SPY

This is a listener and visualizer for reverse-engineering SPI protocol between main CPU and control panel of Pioneer CDJ-1000mk3.

You can see a demonstration [here](https://www.youtube.com/watch?v=yRItR1R8qdQ):

[![View demo on Youtube](https://img.youtube.com/vi/yRItR1R8qdQ/0.jpg)](https://www.youtube.com/watch?v=yRItR1R8qdQ)

## FL Display
![](https://github.com/DjFix/spi-spy/blob/master/Doc/FL%20Display.png)

## Jog Dial Display
![](https://github.com/DjFix/spi-spy/blob/master/Doc/Jog%20Dial%20display.png)

## Upper panel

![](https://github.com/DjFix/spi-spy/blob/master/Doc/Upper%20Panel.png)

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

| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 | 24 | 25 | 26 |
|---|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|
| 1    | 16   | ATB  | ARS  | APH  | APL  | APCH | APCL | JPH  | JPL  | JSH  | JSL  | JST  | 0x00 | B1   | 0x00 | B2   | B3   | B4   | CRC? | 241  | 100 | 118  | 0x00 | 0x00 | 0x00 | 0x00 |

### JST bitmask
| 0  | 1  | 2  | 3  |  4  |  5 |  6 | 7  |
|-----|-----|-----|-----|------|-----|-----|-----|
| Eject Lock switch: 1 = unlock / 0 = lock | Direction switch: 1 = forward / 0 = reverse  | 0 | 0 | touch detect | touch detect |  1 = jog forward rotation / 0 = jog reverse rotation | jog rotation detect |


### B0 bitmask
| 0  | 1  | 2  | 3  |  4  |  5 |  6 | 7  |
|-----|-----|-----|-----|------|-----|-----|-----|
| PLAY | CUE | IN | OUT | Reloop | A | B | C |

### B1 bitmask

| 0  | 1  | 2  | 3  |  4  |  5 |  6 | 7  |
|-----|-----|-----|-----|------|-----|-----|-----|
| REC | \|<< | \>\>\| | << | \>\> | 0 | 0 | 0 |

### B2 bitmask

| 0  | 1  | 2  | 3  |  4  |  5 |  6 | 7  |
|-----|-----|-----|-----|------|-----|-----|-----|
| < MP3  | MP3 \> | JOG MODE | Tempo | Master Tempo | Tempo Reset | 0 | 0 |

### B3 bitmask

| 0  | 1  | 2  | 3  |  4  |  5 |  6 | 7  |
|-----|-----|-----|-----|------|-----|-----|-----|
| < Call | Call \> | Memory | Delete | Eject | Time | Text | 0 |

See [original doc](https://github.com/DjFix/spi-spy/blob/master/Doc/Reverse_Engineering_Pioneer_CDJ-1000_serial_protocol.pdf) for more info

-------
Author of research: [Anatsko Andrei](https://github.com/djgreeb)
