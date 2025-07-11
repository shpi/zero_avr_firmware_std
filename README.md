
# SHPI.zero Basic ATmega32u4 Firmware

Basic firmware for ATmega32u4 as I2C-Slave

- communication only over I2C (USB free for CULFW)
- backlight control
- LCD setup
- control relays
- control internal blower fan
- control LED
- read sensors and more
- uses counter0 for VENT PWM

## prerequisites
A SHPI.zero with GCC and DFU-programmer installed.
```bash
sudo apt-get install dfu-programmer gcc-avr binutils-avr avr-libc
```


## Getting Started

Download GIT folder on your SHPI.zero:
```bash
git clone https://github.com/shpi/zero_firmware_std.git
```


## I2C example commands

For running command on console, u need to disable CRC check.

Disable CRC check

```bash
i2cset -y 2 0x2A 0xFE 0x00             

```

Read relay 1 status

```bash
i2cget -y 2 0x2A 0x0D             

```
it should return :
```
0x00   // or 0xFF, depending relay 1 status 
```

If response consists more bytes, use

```bash
i2cget -y 2 0x2A    

```
to read one more byte.




## Compile & flash

To compile and flash:

```bash
cd zero_firmware_std
sudo make flash
```

Please restart after flashing, to avoid touchscreen issues.


## I2C commands


|Function                |Read addr        |Write addr        |Bytes|Interpretation           |Range                                                                                      |
|------------------------|-----------------|------------------|-----|-------------------------|-------------------------------------------------------------------------------------------|
|Range                   |0x01-0x7F        |0x80-0xFF         |     |                         |                                                                                           |
|                        |                 |                  |     |                         |                                                                                           |
|A0 (GD0 CC1101)   |0x00             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|A1 (MICS CO)            |0x01             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|A2 (MICS No2)           |0x02             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|A3 (MICS NH3            |0x03             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|A4 (MP503)              |0x04             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|A5 (NTC 10K)            |0x05             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|A7 (ACS712)             |0x06             |                  |2    |10bit in 2 Byte, LOW→HIGH|0 -1023                                                                                    |
|BACKLIGHT_LEVEL         |0x07             |0x87              |1    |1 Byte                   |0-31                                                                                       | 
|VENT_RPM                |0x08             |                  |2    |2 Byte                   |UPM                                                                                        |
|VCC ATmega              |0x09             |                  |2    |2 Byte                   |in Millivolts                                                                              |
|Internal Temp ATmega    |0x0A             |                  |2    |2 Byte                   |degree celsius                                                                             |
|FREERAM ATmega                |0x0B             |                  |2    |2 Byte                   |in Bytes                                                                                   |
|LED_COLOR               |0x0C             |              |3    |3 Byte                   |R, G, B → 0-254,0-254,0-254                                                                |
|Relay1                  |0x0D             |0x8D              |1    |1 Byte                   |0x00=off,  0xFF=on                                                                         |
|Relay2                  |0x0E             |0x8E              |1    |1 Byte                   |0x00=off,  0xFF=on                                                                         |
|Relay3                  |0x0F             |0x8F              |1    |1 Byte                   |0x00=off,  0xFF=on                                                                         |
|D13 (reset RPI)         |0x10             |0x90              |1    |1 Byte                   |0x00=off, 0x01=1sec low for reset*, 0xFF=on                                                |
|HWB (5Vheat gasheater)         |0x11             |0x91              |1    |1 Byte                   |0x00=off, 0xFF=on                                                                          |
|Buzzer                  |0x12             |0x92              |1    |1 Byte                   |0x00=off, 0xFF=on, more will follow                                                        |
|VENT_PWM                |0x13             |0x93              |1    |1 Byte                   |0=max speed, ..., 253=minimum,254=autominimum, 255=off                                     |
|LED_COLOR_R             |0x14             |0x94  set R color |1    |1 Byte                   |0 – 255                                                                                    |
|LED_COLOR_G             |0x15             |0x95  set G color |1    |1 Byte                   |0 – 255                                                                                    |
|LED_COLOR_B             |0x16             |0x96  set B color |1    |1 Byte                   |0 – 255                                                                                    |
|A7_AVG (AC curr)          |0x17 (prev.:0x14)|                  |2    |2 Byte                   |0 – 1023                                                                                   |
|DISPLAY CONTROLLER      |0x18             |0x98              |1    |1 Byte                   |0x00=off, 0xFF=on                                                                          |
|ALL PIXEL ON / OFF      |0x19             |0x99              |1    |1 Byte                   |0x00=off, 0x01=normal , 0xFF=on                                                                          |
|WATCHDOG                |0x20             |0xA0              |1    |1 Byte                   |0x00=off, 0x01=only LED, 0xF1=hard reset, 0xFF= hard reset, with fallback                  |
|RGB led control position|0x21             |0xA1              |1    |1 Byte                   |0x00=LOGO, 0x01=signal LED                                                                 |
|DFU BOOT                    |                 |0xFD              |1    |1 Command                |0xFF=DFU active                                                                            |
|CRC CHECK                  |0x7e             |0xFE              |1    |1 Byte                   |0x00=off, 0xFF=on                                                                          |
|FW_VERSION              |0x7F             |                  |1    |1 Byte                   |0xFF=Pre 2.0, 0x01=2.0                                                                     |  








## License

This project is licensed under the GPL v3 license.
