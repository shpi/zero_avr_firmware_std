# SHPI.zero Basic ATmega32u4 Firmware

basic firmware for:

- backlight control
- lcd setup
- control relais
- control vent
- read sensors

## prerequisites
A SHPI.zero with GCC and dfu-programmer installed.

 installed

## Getting Started

Download git folder on your SHPI.zero:
```bash
git clone https://github.com/shpi/zero_avr_firmware_std.git
```



I2C command test:
```bash
i2cget -y 2 0x2A 0x0D   //0x0D  read relay 1          

```
it should return :
```
0x00   // or 0xFF, depending relay 1 status 
```


## Usage
To compile and flash:
```bash
cd zero_avr_firmware
sudo make flash
```



## License

This project is licensed under the GPL v3 license.
