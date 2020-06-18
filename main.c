/*
*  SHPI.zero Basic Firmware v1.2 BETA 
*  
* 
*
* Basic Firmware for ATmega32u4 slave,  no radio module support included
*
* Jul    15th, 2019  v 1.1b initial Version

* October 4th, 2019  v1.2  Beta
*    NEW: implemented CRC Check via CRC8  CCITT
*
*
*   I2C Command Reference     (SLAVE ADDRESS 0x2A)
*
*   ACTIVE COMMANDS
*
*   0x87 set backlight level of LCD   0-31  // i2cset -y 2 0x2A 0x87 31 +CRC
*OLD:  0x8C  set RGB value of LED              // i2cset -y 2 0x2A 0x8C 0xRR 0xGG 0xBB +CRC i         (replace RR GG BB by value 0-255)
*   0x8D  set Relay 1                       // i2cset -y 2 0x2A 0x8D 0x00 +CRC                     (0x00 = off  0xFF = on)
*   0x8E  set Relay 2                       // i2cset -y 2 0x2A 0x8E 0x00 +CRC                    (0x00 = off  0xFF = on)
*   0x8F  set Relay 3                       // i2cset -y 2 0x2A 0x8F 0x00 +CRC                    (0x00 = off  0xFF = on)
*   0x90  set D13 / PC7                     // i2cset -y 2 0x2A 0x90 0x00 +CRC                    (0x00 = off  0xFF = on)
*   0x91  set HWB / PE2                     // i2cset -y 2 0x2A 0x91 0x00 +CRC                    (0x00 = off  0xFF = on)
*   0x92  set Buzzer / PB5                  // i2cset -y 2 0x2A 0x92 0x00 +CRC                    (0x00 = off, 0x01 on for click sound,0xFF = on)
*   0x93  set Vent power                    // i2cset -y 2 0x2A 0x93 0x00 +CRC                    (PWM:  0x00 = on ... 0xFF = off)
*   0x94  set R color
*   0x95  set G color
*   0x96  set B color 
*
*
*  READOUT VALUES
*
*
*   0x00  read A0                           // i2cset -y 2 0x2A 0x00                    
*                                           // i2cget -y 2 0x2A         -> frst low byte
*                                           // i2cget -y 2 0x2A         -> scnd high byte
*
*   0x01  read A1 
*   0x02  read A2 
*   0x03  read A3 
*   0x04  read A4 
*   0x05  read A5 
*   0x06  read A7 
*
*   0x07  read actual backlight level       // i2cget -y 2 0x2A 0x07
*                                             -> only one byte response
*
*
*   0x08  read actual RPM von vent         // i2cset -y 2 0x2A 0x08
*                                          // i2cget -y 2 0x2A         -> frst low byte
*                                          // i2cget -y 2 0x2A         -> scnd high byte
*
*   0x09 read supply voltage of ATmega     // i2cset -y 2 0x2A 0x09                    
*                                          // i2cget -y 2 0x2A         -> frst low byte
*                                          // i2cget -y 2 0x2A         -> scnd high byte
*
*
*
*
*   0x0A read internal ATmega temp         // i2cset -y 2 0x2A 0x0A                    
*                                          // i2cget -y 2 0x2A         -> frst low byte
*                                          // i2cget -y 2 0x2A         -> scnd high byte
*
*
*   0x0B read free RAM ATmega              // i2cset -y 2 0x2A 0x0B                    
*                                          // i2cget -y 2 0x2A         -> frst low byte
*                                          // i2cget -y 2 0x2A         -> scnd high byte
*
*
*   0x0C read RGB values from LED           // i2cset -y 2 0x2A 0x0C                    
*                                          // i2cget -y 2 0x2A         -> red byte
*                                          // i2cget -y 2 0x2A         -> green byte
*                                          // i2cget -y 2 0x2A         -> blue byte
*
*
*                                         
*   0x0D  read relay 1                      // i2cget -y 2 0x2A 0x0D 
*   0x0E  read relay 2                      // i2cget -y 2 0x2A 0x0E 
*   0x0F  read relay 3                      // i2cget -y 2 0x2A 0x0F 
*   0x10  read D13                         // i2cget -y 2 0x2A 0x10 
*   0x11  read HWB                         // i2cget -y 2 0x2A 0x11 
*   0x12  read Buzzer                      // i2cget -y 2 0x2A 0x12 
*   0x13  read VENT PWM value              // i2cget -y 2 0x2A 0x13    
*   0x14  read AVG A7 for AC currents 
*
*
*
*
*
* Author: Lutz Harder
* License: GNU GPL
*/

#include <util/crc16.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/io.h>
#include <stdlib.h>
#define DELAY 100
#define WAIT 120
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/twi.h>
#define ws2812_resettime  300 
#define ws2812_port D   
#define ws2812_pin  5   
#include "light_ws2812.c"
#include "light_ws2812.h"
struct cRGB led[1];
#define I2C_ADDR 0x2A
#define SDA_LINE  (PIND & (1<<PD1))

uint8_t commandbyte = 0xFF,twdrbuffer, buffer_address,a7count = 0,count,bllevel = 31,newbllevel = 31,changeled,crc,i2cerror = 0;  
uint16_t a0,a1,a2,a3,a4,a5,a7,a7avg,a7max,a7min,vcc,temp,rpm,fanspin,isrtimer,i2cbuffer = 0;
uint8_t fanlevel = 254;



uint16_t commands[] =  {0x0080, 0x01F8,  0x0246, 0x0305, 0x0440,  0x0540, 0x0640,0x0740, 0x0840,  0x0940, 0x0A03};            // only for  A035VW01 
uint16_t commands2[]=  {0x0011,0x0000,0x0001,0x0000,0x00C1,0x01A8,0x01B1,0x0145,0x0104,0x0C5,0x0180,0x016C,0x0C6,               // initcode for LCD A035VL01
                       0x01BD,0x0184,0x00C7,0x01BD,0x0184,0x00BD,0x0102,0x0011,0x0000,0x00F2,0x0100,0x0100,0x0182,
                       0x0026,0x0108,0x00E0,0x0100,0x0104,0x0108,0x010B,0x010C,0x0111,0x010D,0x010E,0x0100,0x0104,
                       0x0108,0x0113,0x0114,0x012F,0x0129,0x0124,0x00E1,0x0100,0x0104,0x0108,0x010B,0x010C,0x0111,
                       0x010D,0x010E,0x0100,0x0104,0x0108,0x0113,0x0114,0x012F,0x0129,0x0124,0x0026,0x0108,0x00FD,
                       0x0100,0x0108,0x0029};             
            

void writebl(uint8_t data) { // set single wire brightness  AL3050 
  uint8_t count = 8;
  do {
    PORTD &= ~_BV(PD4);
    _delay_us(100);
    if (!(data & (1 << (count - 1)))) {
      _delay_us(100);
    }
    PORTD |= _BV(PD4);
    _delay_us(100);
    if ((data & (1 << (count - 1))) != 0) {
      _delay_us(100);
    }
    count--;
  } while (count);

  PORTD &= ~_BV(PD4);
  _delay_us(100);
  PORTD |= _BV(PD4);
  _delay_us(100);
}

void initbl() { // init AL3050 single wire dimming
  PORTD &= ~_BV(PD4);
  _delay_us(3000);
  PORTD |= _BV(PD4);
  _delay_us(120);
  PORTD &= ~_BV(PD4);
  _delay_us(500);
  PORTD |= _BV(PD4);
  _delay_us(5);
}

void write(uint16_t data, uint8_t count) { //  write routine for LCD setup
  PORTD &= ~_BV(PD4);
  do {
    PORTB &= ~_BV(PB2);
    PORTB |= (((data & (1 << (count - 1))) != 0) << 2); // BITWISE AND -> PB2           
    PORTB &= ~_BV(PB1);
    _delay_us(DELAY);
    PORTB |= _BV(PB1);
    _delay_us(DELAY);
    count--;
  } while (count);
  PORTB &= ~_BV(PB2);
  PORTD |= _BV(PD4);
}

void setup_lcd(void) {
  for (int x = 0; x < 11; x++) {
    write(commands[x], 16);
  } //only for A035VW01
  for (int x = 0; x < (sizeof(commands2) / sizeof(uint16_t)); x++) {
    if (commands2[x] == 0x0000) {
      _delay_ms(WAIT);
      continue;
    }
    write(commands2[x], 9);
  } //only for A035VL01

}

uint16_t readAna(uint8_t channel) {
  uint8_t low, high;
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  ADCSRB = 0x40;
  ADMUX = ((0 << REFS1) | (1 << REFS0) | (0 << ADLAR));

  if (channel >= 8) //
  {
    channel -= 0x08; //ch - 8           
    ADCSRB |= (1 << MUX5); // set MUX5 on ADCSRB to read upper bit ADC8-ADC13
  } else {
    ADCSRB &= ~(1 << MUX5); // clear MUX 5 
  }
  channel &= 0x07;
  ADMUX |= channel; // selecting channel

  ADCSRA |= _BV(ADEN);
  _delay_ms(2);
  ADCSRA |= (1 << ADSC);

  while ((ADCSRA & _BV(ADSC))); // measuring 
  low = ADCL;
  high = ADCH;
  return (high << 8) | low;

}

uint16_t readVcc(void) {
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADEN);
  ADCSRB &= ~_BV(MUX5);
  _delay_ms(2);
  ADCSRA |= 1 << ADSC;
  while ((ADCSRA & _BV(ADSC))); // measuring
  ADCSRA |= 1 << ADSC;
  while ((ADCSRA & _BV(ADSC)));
  return 1125300L / (ADCL | (ADCH<<8));
}

uint16_t GetTemp(void) {

  ADMUX = _BV(REFS1) | _BV(REFS0) | 7; // Set internal V reference, temperature reading
  ADCSRB = 0x20; // ref  24.6
  ADCSRA &= ~(_BV(ADATE) | _BV(ADIE)); // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN); // enable the ADC
  _delay_ms(2); // delay for voltages to become stable.

  ADCSRA |= _BV(ADSC); // measuring
  while ((ADCSRA & _BV(ADSC)));

  ADCSRA |= _BV(ADSC);
  while ((ADCSRA & _BV(ADSC)));

  return (ADCL | (ADCH << 8));
}

uint16_t freeRam(void) {
  extern char __heap_start, * __brkval;
  int v;
  return (uint16_t) & v - (__brkval == 0 ? (int) & __heap_start : (int) __brkval);
}

void I2C_init(uint8_t address) // setup ATmega as I2C slave
{
  cli();

  TWAR = (address << 1);
  TWCR = (1 << TWEN) | // TWI Interface enabled.
    (1 << TWIE) | (1 << TWINT) | // Enable TWI Interupt and clear the flag.
    (1 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // Prepare to ACK next time the Slave is addressed.
    (0 << TWWC);

  buffer_address = 0xFF;

}

ISR(PCINT0_vect) {
  sei();
  if (bit_is_clear(PINB, PB0)) fanspin++;
} // counting VENT_RPM

ISR(TIMER0_OVF_vect) {
  isrtimer++;
} // reuse timer0 for counting VENT_RPM

ISR(TWI_vect) {

  switch (TW_STATUS) {

  case TW_SR_SLA_ACK:

    TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
    buffer_address = 0xFF; // set buffer pos undefined
    break;
   
  case TW_SR_DATA_ACK: // received data from master
    
    if (buffer_address == 0xFF) {

      commandbyte = TWDR;
      crc = _crc8_ccitt_update(0, commandbyte);
      buffer_address = 0;
      i2cerror = 0;


      switch (commandbyte) {		 

		  case 0x00: i2cbuffer = a0; break;
		  case 0x01: i2cbuffer = a1; break;
		  case 0x02: i2cbuffer = a2; break;
		  case 0x03: i2cbuffer = a3; break;
		  case 0x04: i2cbuffer = a4; break;
		  case 0x05: i2cbuffer = a5; break;
		  case 0x06: i2cbuffer = a7; break;
		  case 0x08: i2cbuffer = rpm;break;
		  case 0x09: i2cbuffer = vcc; break;
		  case 0x0A: i2cbuffer = temp;break;
		  case 0x0B: i2cbuffer = freeRam(); break;
		  case 0x14: i2cbuffer = a7avg; break;


	    }
      } 
      else { 

      
 
      if (buffer_address == 0) {twdrbuffer = TWDR; crc = _crc8_ccitt_update(crc,TWDR);}

      else  if ((buffer_address == 1) & (TWDR == crc)) {

           if (commandbyte == 0x87 ) {newbllevel = twdrbuffer;}
      else if (commandbyte == 0x8D ) {if (twdrbuffer == 0xFF) {PORTC |= _BV(PC6);} else {PORTC &= ~_BV(PC6); }}  //set Relais 1
      else if (commandbyte == 0x8E ) {if (twdrbuffer == 0xFF) {PORTB |= _BV(PB4);} else {PORTB &= ~_BV(PB4); }}  //set Relais 2
      else if (commandbyte == 0x8F ) {if (twdrbuffer == 0xFF) {PORTB |= _BV(PB6);} else {PORTB &= ~_BV(PB6); }} //set Relais 3
      else if (commandbyte == 0x90 ) {if (twdrbuffer == 0xFF) {PORTC |= _BV(PC7);} else {PORTC &= ~_BV(PC7); }} //set D13
      else if (commandbyte == 0x91 ) {if (twdrbuffer == 0xFF) {PORTE |=  (1<<2);}  else {PORTE &= ~(1<<2);   }}     //set HWB ->Gasheater      (D13 on prototypes)
      else if (commandbyte == 0x92 ) {if (twdrbuffer == 0xFF) {PORTB |= _BV(PB5);} else if (twdrbuffer == 0x01) {PORTB |= _BV(PB5); twdrbuffer = 0x02;} else {PORTB &= ~_BV(PB5);twdrbuffer = 0x00;}}   //set Buzzer
      else if (commandbyte == 0x93 ) {OCR0A = twdrbuffer;fanlevel = twdrbuffer;}  //set Vent
      else if (commandbyte == 0x94 ) {led[0].r = twdrbuffer;changeled = 1;}  //set r color
      else if (commandbyte == 0x95 ) {led[0].g = twdrbuffer;changeled = 1;}  //set g color
      else if (commandbyte == 0x96 ) {led[0].b = twdrbuffer;changeled = 1;}  //set b color

      else {i2cerror++;} 
      } 
      else {i2cerror++;}

      buffer_address++;

      }
      TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
      if ((commandbyte == 0x92) & (twdrbuffer == 0x02)) {_delay_us(25); PORTB &= ~_BV(PB5);}
      break;

    case TW_ST_SLA_ACK: //  slave adressed
    case TW_ST_DATA_ACK:
      
      _delay_us(0.3); 

	     switch(commandbyte)  {		 

                 case 0x87:
                 case 0x8D:
                 case 0x8E:
                 case 0x8F:
                 case 0x90:
                 case 0x91:
                 case 0x92:
                 case 0x93:
                 case 0x94:
                 case 0x95:
                 case 0x96:  { TWDR = crc;  crc = 0xFF;} break;
                 case 0x00:
                 case 0x01:
                 case 0x02:
                 case 0x03:
                 case 0x04:
                 case 0x05:
                 case 0x06:
                 case 0x08: 
                 case 0x09: 
                 case 0x0A: 
                 case 0x0B: 
                 case 0x14:      
                                 if (buffer_address == 0) {TWDR = i2cbuffer & 0xFF; crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = i2cbuffer >> 8;   crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 2) {TWDR = crc;}
                            else                          {TWDR = 0xFF; i2cerror++;}
                            break; 
                           
                 case 0x0C:     
                                if (buffer_address == 0)  {TWDR = led[0].r; crc = _crc8_ccitt_update(crc,TWDR);}  
                           else if (buffer_address == 1)  {TWDR = led[0].g; crc = _crc8_ccitt_update(crc,TWDR);}
                           else if (buffer_address == 2)  {TWDR = led[0].b; crc = _crc8_ccitt_update(crc,TWDR);}
                           else if (buffer_address == 3)  {TWDR = crc;}
                           else                           {TWDR = 0xFF;  i2cerror++;}
                           break; 

                 case 0x07:          
                                     if (buffer_address == 0) {TWDR = bllevel; crc = _crc8_ccitt_update(crc,TWDR);}
                                else if (buffer_address == 1) {TWDR = crc;}
                                else                         {TWDR = 0xFF; i2cerror++;}
                                break;


                 case 0x0D:      
                                 if (buffer_address == 0) {if (bit_is_set(PINC,PC6)) {TWDR = 0xFF;} else {TWDR = 0x00;} crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = crc;}
                            else {TWDR = 0xFF; i2cerror++;}
                            break;

                 case 0x0E:     
                                 if (buffer_address == 0) {if (bit_is_set(PINB,PB4)) {TWDR = 0xFF;} else {TWDR = 0x00;} crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = crc;}
                            else                          {TWDR = 0xFF; i2cerror++;}
                            break;


                 case 0x0F:     
                                 if (buffer_address == 0) {if (bit_is_set(PINB,PB6)) {TWDR = 0xFF;} else {TWDR = 0x00;} crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) { TWDR = crc;}
                            else                          {TWDR = 0xFF; i2cerror++;}
                            break;

                 case 0x10:  
                                 if (buffer_address == 0) {if (bit_is_set(PINC,PC7)) {TWDR = 0xFF;} else {TWDR = 0x00;} crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = crc;}
                            else                          {TWDR = 0xFF; i2cerror++;}
                            break;


                 case 0x11:  
                                 if (buffer_address == 0) {if (bit_is_set(PINE,PE2)) {TWDR = 0xFF;} else {TWDR = 0x00;} crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = crc;}
                            else                          {TWDR = 0xFF; buffer_address = 0xFE; i2cerror++;}
                            break;


                 case 0x12:  
                                 if (buffer_address == 0)  {if (bit_is_set(PINB,PB5)) {TWDR = 0xFF;} else {TWDR = 0x00;} crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = crc;}
                            else                          {TWDR = 0xFF; i2cerror++;}
                            break;


                 case 0x13:  
                                 if (buffer_address == 0) {TWDR = OCR0A; crc = _crc8_ccitt_update(crc,TWDR);}
                            else if (buffer_address == 1) {TWDR = crc;}
                            else                          {TWDR = 0xFF; i2cerror++;}
                            break;


                 default: TWDR = 0xFF; 

		 }	 



      
      buffer_address++;
      TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);        
      break;                                 
   
    case TW_BUS_ERROR:   
     TWCR =   (1<<TWSTO)|(1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
     break;


    //case TW_SR_STOP:  TWCR |= (1<<TWINT)|(1<<TWEA)|(1<<TWEN);  break;

    default:         
      TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)| (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|  (0<<TWWC);   
                                                                                                    
      
  }



}


void setup(void)
{
   DDRF = 0b00000000;
   DDRD = 0b01111000;
   PORTD= 0b00000000;
   DDRE = 0b00000000;   // DDRE |= (1<<2);   be carefull with hwb, check if its connected to GND via 10k (prototypes!)
   DDRB = 0b11110110;
   DDRC = 0b11000000;
   OCR0A = 0;           //    start value for FAN  0 / 255  (-> p-channel so inverted)       0x00 is ON  0xFF is OFF
   TCCR0B  =  0b00000001;
   TCCR0A  =  0b10000011;            // 8bit  62khz
   TIMSK0 |= (1 << TOIE0);            // init interrupt for timer0 overflow
   clock_prescale_set(clock_div_1);
   I2C_init(I2C_ADDR);
   PCICR |= _BV(PCIE0);              // enable pin change interrupt for PB0 (rpm)
   PCMSK0 |= _BV(PCINT0);
   sei();
   led[0].r = 255;
   led[0].g = 255;
   led[0].b = 255;
   ws2812_setleds(led,1);
   setup_lcd();
   initbl();
   led[0].r = 0;
   led[0].g = 0;
   led[0].b = 0;
   ws2812_setleds(led,1);
   OCR0A = 210;
   wdt_enable(WDTO_8S);
   bllevel = 0;
   newbllevel = 31;
}


int main(void)
{
  uint8_t adcselect = 0;
  setup();

  while(1) {  

  if (!SDA_LINE) {i2cerror++;}
  
  if (i2cerror > 50) {
  TWCR =   (1<<TWSTO)|(1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (0<<TWEN); 
  I2C_init(I2C_ADDR); 
  i2cerror = 0;
  sei();
  }                    

  wdt_reset();	
  if (isrtimer > 31250)   // routine for calculate fan speed - timer is 64khz
  {rpm = fanspin * 30;    // 2 signals each turn        
  fanspin = 0;
  isrtimer = 0;
  if (fanlevel == 254) { //fan minimal auto
  if (rpm > 1950) {OCR0A++;}
  if (rpm < 1800) {OCR0A--;}
  }
  }

  if (changeled)  {
      ws2812_setleds(led,1);
      changeled = 0;
                     }

  if (0 <= newbllevel && newbllevel < 32) {
  if (newbllevel < bllevel) {                // smooth backlight level change in steps

      bllevel--;
      writebl(0b01011000);   writebl(0b00011111 & bllevel);
	  }

     if (newbllevel > bllevel) {

	  bllevel++;
	   writebl(0b01011000);   writebl(0b00011111 & bllevel);

	  }}

  if (adcselect < 10) {adcselect++;} else {adcselect = 0;}
  
  switch(adcselect)
  {
   case 0: a0 = readAna(7);  break;
   case 1: a1 = readAna(6);  break;
   case 2: a2 = readAna(5);  break;
   case 4: a3 = readAna(4);  break; 
   case 5: a4 = readAna(1);  break; 
   case 7: a5 = readAna(0);  break;
   case 8: vcc = readVcc();  break;
   case 10: temp = GetTemp();   break;

 
   default: {a7 = readAna(10);  //read A7 more frequently 
           if (a7 > a7max) a7max = a7;
           if (a7 < a7min) a7min = a7;
           a7count++;
           if (a7count > 60) {a7avg = (a7max - ((a7max +  a7min)/ 2)) * 0.707 ; a7min = 1024; a7max = 0; a7count = 0;} 
           break; 
           }

}}}
