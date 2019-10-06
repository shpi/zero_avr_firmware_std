/*
*  SHPI.zero Basic Firmware v1.1 BETA
*
* Basic Firmware for ATmega32u4 slave,  no radio module support included
*
* Jul 15th, 2019  v1.1b initial Version
*
*
*   I2C Command Reference     (SLAVE ADDRESS 0x2A)
*
*   ACTIVE COMMANDS
*
*   0x87 set backlight level of LCD   0-31  // i2cset -y 2 0x2A 0x87 31
*   0x8C  set RGB value of LED              // i2cset -y 2 0x2A 0x8C 0xRR 0xGG 0xBB i          (replace RR GG BB by value 0-255)
*   0x8D  set Relay 1                       // i2cset -y 2 0x2A 0x8D 0x00                     (0x00 = off  0xFF = on)
*   0x8E  set Relay 2                       // i2cset -y 2 0x2A 0x8E 0x00                     (0x00 = off  0xFF = on)
*   0x8F  set Relay 3                       // i2cset -y 2 0x2A 0x8F 0x00                     (0x00 = off  0xFF = on)
*   0x90  set D13 / PC7                     // i2cset -y 2 0x2A 0x90 0x00                     (0x00 = off  0xFF = on)
*   0x91  set HWB / PE2                     // i2cset -y 2 0x2A 0x91 0x00                     (0x00 = off  0xFF = on)
*   0x92  set Buzzer / PB5                  // i2cset -y 2 0x2A 0x92 0x00                     (0x00 = off, 0x01 on for click sound,0xFF = on)
*   0x93  set Vent power                    // i2cset -y 2 0x2A 0x93 0x00                     (PWM:  0x00 = on ... 0xFF = off)
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
#define SCL_LINE  (PIND & (1<<PD0))
#define SDA_LINE  (PIND & (1<<PD1))


uint8_t commandbyte, buffer_address,a7count = 0,count,bllevel = 31,newbllevel = 31,changeled;  
uint16_t a0,a1,a2,a3,a4,a5,a7,a7avg,a7max,a7min,vcc,temp,rpm,fanspin,isrtimer;




uint16_t commands[] =  {0x0080, 0x01F8,  0x0246, 0x0305, 0x0440,  0x0540, 0x0640,0x0740, 0x0840,  0x0940, 0x0A03};            // only for  A035VW01 
uint16_t commands2[]=  {0x0011,0x0000,0x0001,0x0000,0x00C1,0x01A8,0x01B1,0x0145,0x0104,0x0C5,0x0180,0x016C,0x0C6,               // initcode for LCD A035VL01
                       0x01BD,0x0184,0x00C7,0x01BD,0x0184,0x00BD,0x0102,0x0011,0x0000,0x00F2,0x0100,0x0100,0x0182,
                       0x0026,0x0108,0x00E0,0x0100,0x0104,0x0108,0x010B,0x010C,0x0111,0x010D,0x010E,0x0100,0x0104,
                       0x0108,0x0113,0x0114,0x012F,0x0129,0x0124,0x00E1,0x0100,0x0104,0x0108,0x010B,0x010C,0x0111,
                       0x010D,0x010E,0x0100,0x0104,0x0108,0x0113,0x0114,0x012F,0x0129,0x0124,0x0026,0x0108,0x00FD,
                       0x0100,0x0108,0x0029};             
            

void writebl(uint8_t data) {            // set single wire brightness  AL3050 
    uint8_t count = 8;
    do {  
    PORTD &= ~_BV(PD4);
    _delay_us(100);
    if (!(data & (1 << (count-1)))) { _delay_us(100);  }
    PORTD |= _BV(PD4);
    _delay_us(100);
     if ((data & (1 << (count-1))) != 0) { _delay_us(100);  }
    count --;
    } while (count) ;      
    
    PORTD &= ~_BV(PD4);
    _delay_us(100);
    PORTD |= _BV(PD4);
    _delay_us(100);
}



void initbl(){              // init AL3050 single wire dimming
PORTD &= ~_BV(PD4);
_delay_us(3000);
PORTD |= _BV(PD4);
_delay_us(120);
PORTD &= ~_BV(PD4);
_delay_us(500);
PORTD |= _BV(PD4);
_delay_us(5);
}



void write(uint16_t data, uint8_t count){                                   //  write routine for LCD setup
    PORTD &= ~_BV(PD4);
    do {
        PORTB &= ~_BV(PB2);
        PORTB |= (((data & (1 << (count-1))) != 0) << 2);       // BITWISE AND -> PB2           
        PORTB &= ~_BV(PB1);
        _delay_us(DELAY);
        PORTB |= _BV(PB1);
        _delay_us(DELAY);
        count--;
    }  while (count);
    PORTB &= ~_BV(PB2);
    PORTD |= _BV(PD4);
}

void setup_lcd(void){
   for(int x = 0; x < 11; x++){write(commands[x], 16);}        //only for A035VW01
   for(int x=0;x < (sizeof(commands2) / sizeof(uint16_t)); x++){if(commands2[x]== 0x0000){_delay_ms(WAIT);continue;} write(commands2[x],9);  }      //only for A035VL01
    
}

  


uint16_t readAna(uint8_t channel) {
 uint8_t low, high;
 ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
 ADCSRB = 0x40; 
 ADMUX  = ((0<<REFS1)|	  (1<<REFS0)|  (0<<ADLAR));

 if (channel >=8)//
  {
    channel -= 0x08;//ch - 8           
    ADCSRB |=  (1 << MUX5);   // set MUX5 on ADCSRB to read upper bit ADC8-ADC13
  } 
  else
  {    
    ADCSRB &=  ~(1 << MUX5);   // clear MUX 5 
  }
  channel &= 0x07; 
  ADMUX |= channel; // selecting channel
     


 ADCSRA |= _BV(ADEN);
 _delay_ms(3); 
  ADCSRA |= (1 << ADSC);

  while((ADCSRA & _BV(ADSC)));  // measuring 
  low  = ADCL;
  high = ADCH;  
  return (high << 8) | low;

}


uint16_t readVcc(void) {
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADEN);    
  ADCSRB &= ~_BV(MUX5);
  _delay_ms(3);   
  ADCSRA |= 1 << ADSC;
  while((ADCSRA & _BV(ADSC)));  // measuring
  ADCSRA |= 1 << ADSC;
  while((ADCSRA & _BV(ADSC))); 
  return 1125300L / (ADCL | (ADCH<<8));
}


uint16_t GetTemp(void)
{

  ADMUX = _BV(REFS1) | _BV(REFS0) | 7;   // Set internal V reference, temperature reading
  ADCSRB = 0x20;                          // ref  24.6
  ADCSRA &= ~(_BV(ADATE) |_BV(ADIE));   // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN);                   // enable the ADC
  _delay_ms(3);                       // delay for voltages to become stable.

 ADCSRA |= _BV(ADSC);                 // measuring
 while((ADCSRA & _BV(ADSC)));             

 ADCSRA |= _BV(ADSC); 
 while((ADCSRA & _BV(ADSC)));             

 return (ADCL | (ADCH << 8));
}


   

uint16_t freeRam () {
  extern char __heap_start, *__brkval;
  int v;
  return (uint16_t) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


void I2C_init(uint8_t address)             // setup ATmega as I2C slave
{
  cli();
  TWAR = address << 1;
  TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
  buffer_address = 0xFF;
  count = 0xFF;
}

ISR(PCINT0_vect) {if (bit_is_clear(PINB,PB0)) fanspin++; }  // counting VENT_RPM

ISR(TIMER0_OVF_vect){ isrtimer++; }  // reuse timer0 for counting VENT_RPM


ISR(TWI_vect)
{

  

  switch(TW_STATUS)
  {

     case TW_SR_SLA_ACK: //  slave adressed
	    TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	    buffer_address=0xFF; // set buffer pos undefined
      break;

    case TW_SR_DATA_ACK:
      // received data from master
      if (buffer_address == 0xFF) {
      commandbyte  = TWDR;   buffer_address = 0;  count = 0;} 
      else {
      buffer_address++; 
           if ((commandbyte == 0x8C ) & (buffer_address == 1)) {led[0].r = TWDR;           }             // set RGB values for LED
      else if ((commandbyte == 0x8C ) & (buffer_address == 2)) {led[0].g = TWDR;           }
      else if ((commandbyte == 0x8C ) & (buffer_address == 3)) {led[0].b = TWDR; changeled = 1;} 
                                                                                                        
    
      
      else if ((commandbyte == 0x87 ) & (buffer_address == 1)) {newbllevel = TWDR; }  
      else if ((commandbyte == 0x8D ) & (buffer_address == 1)) {if (TWDR == 0xFF) {PORTC |= _BV(PC6);} else {PORTC &= ~_BV(PC6);} }  //set Relais 1
      else if ((commandbyte == 0x8E ) & (buffer_address == 1)) {if (TWDR == 0xFF) {PORTB |= _BV(PB4);} else {PORTB &= ~_BV(PB4);} }  //set Relais 2
      else if ((commandbyte == 0x8F ) & (buffer_address == 1)) {if (TWDR == 0xFF) {PORTB |= _BV(PB6);} else {PORTB &= ~_BV(PB6);} }  //set Relais 3
      else if ((commandbyte == 0x90 ) & (buffer_address == 1)) {if (TWDR == 0xFF) {PORTC |= _BV(PC7);} else {PORTC &= ~_BV(PC7);} }  //set D13
      else if ((commandbyte == 0x91 ) & (buffer_address == 1)) {if (TWDR == 0xFF) {PORTE |=  (1<<2);} else {PORTE &= ~(1<<2);} }  //set HWB ->Gasheater      (D13 on prototypes)
      else if ((commandbyte == 0x92 ) & (buffer_address == 1)) {if (TWDR == 0xFF) {PORTB |= _BV(PB5);} else if (TWDR ==0x01) {PORTB |= _BV(PB5);} else {PORTB &= ~_BV(PB5);} }  //set Buzzer
      else if ((commandbyte == 0x93 ) & (buffer_address == 1)) {OCR0A = TWDR;}  //set Vent
      
      
      
      } 
      
      TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
      if ((commandbyte == 0x92) & (buffer_address == 1) & (TWDR == 0x01)) {;_delay_us(30); PORTB &= ~_BV(PB5);}

      
      break;
    case TW_ST_SLA_ACK:

           if ((commandbyte == 0x00) & (count == 0))  {TWDR = a0 & 0xFF;}               // write A0 to master
      else if ((commandbyte == 0x00) & (count == 1))  {TWDR = a0 >> 8;}
 
      else if ((commandbyte == 0x01) & (count == 0))  {TWDR = a1 & 0xFF;}               // write A1 to master
      else if ((commandbyte == 0x01) & (count == 1))  {TWDR = a1 >> 8;}
 
      else if ((commandbyte == 0x02) & (count == 0))  {TWDR = a2 & 0xFF;}               // write A2 to master
      else if ((commandbyte == 0x02) & (count == 1))  {TWDR = a2 >> 8;}
 
      else if ((commandbyte == 0x03) & (count == 0))  {TWDR = a3 & 0xFF;}               // write A3 to master
      else if ((commandbyte == 0x03) & (count == 1))  {TWDR = a3 >> 8;}
      
      else if ((commandbyte == 0x04) & (count == 0))  {TWDR = a4 & 0xFF;}              // write A4 to master , standard Winsen MP135
      else if ((commandbyte == 0x04) & (count == 1))  {TWDR = a4 >> 8;}
      
      else if ((commandbyte == 0x05) & (count == 0))  {TWDR = a5 & 0xFF;}              // write A5 to master
      else if ((commandbyte == 0x05) & (count == 1))  {TWDR = a5 >> 8;}
      
      else if ((commandbyte == 0x06) & (count == 0))  {TWDR = a7 & 0xFF;}             // write A7 to master
      else if ((commandbyte == 0x06) & (count == 1))  {TWDR = a7 >> 8;}
      
      else if ((commandbyte == 0x07) & (count == 0))  {TWDR = bllevel;}                // actual backlight level

      else if ((commandbyte == 0x08) & (count == 0))  {TWDR =(rpm & 0xFF);}             // Vent RPM
      else if ((commandbyte == 0x08) & (count == 1))  {TWDR = (rpm >> 8);}

      else if ((commandbyte == 0x09) & (count == 0))  {TWDR = vcc & 0xFF;}           //ATmega32u4 internal vcc
      else if ((commandbyte == 0x09) & (count == 1))  {TWDR = vcc >> 8;}

      else if ((commandbyte == 0x0A) & (count == 0))  {TWDR = temp & 0xFF;}         // ATmega32u4 temp internal
      else if ((commandbyte == 0x0A) & (count == 1))  {TWDR = temp >> 8;}
     
      else if ((commandbyte == 0x0B) & (count == 0))  {TWDR = freeRam() & 0xFF;}   //get available free Ram
      else if ((commandbyte == 0x0B) & (count == 1))  {TWDR = freeRam() >> 8;}
 
      else if ((commandbyte == 0x0C) & (count == 0))  {TWDR = led[0].r;}  //RGB LED    value
      else if ((commandbyte == 0x0C) & (count == 1))  {TWDR = led[0].g;}
      else if ((commandbyte == 0x0C) & (count == 2))  {TWDR = led[0].b;}
  
      else if ((commandbyte == 0x0D) & (count == 0)) {if (bit_is_set(PINC,PC6)) {TWDR = 0xFF;} else {TWDR = 0x00;}}   //read Relais 1 status
      else if ((commandbyte == 0x0E) & (count == 0)) {if (bit_is_set(PINB,PB4)) {TWDR = 0xFF;} else {TWDR = 0x00;}}   //read Relais 2 status
      else if ((commandbyte == 0x0F) & (count == 0)) {if (bit_is_set(PINB,PB6)) {TWDR = 0xFF;} else {TWDR = 0x00;}}   //read Relais 3 status
      else if ((commandbyte == 0x10) & (count == 0)) {if (bit_is_set(PINC,PC7)) {TWDR = 0xFF;} else {TWDR = 0x00;}}   //read D13
      else if ((commandbyte == 0x11) & (count == 0)) {if (bit_is_set(PINE,PE2)) {TWDR = 0xFF;} else {TWDR = 0x00;}}   //read HWB
      else if ((commandbyte == 0x12) & (count == 0)) {if (bit_is_set(PINB,PB5)) {TWDR = 0xFF;} else {TWDR = 0x00;}}   //read buzzer
      else if ((commandbyte == 0x13) & (count == 0))  {TWDR = OCR0A;}   //read vent pwm
   
      else if ((commandbyte == 0x14) & (count == 0))  {TWDR = a7avg & 0xFF;}  
      else if ((commandbyte == 0x14) & (count == 1))  {TWDR = a7avg >> 8;}
 
      else TWDR = 0xFF;
      count++;
      _delay_us(1);  //debugging wait, sometimes raspberry dont see first bit
      TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);

      break;
     

    case TW_BUS_ERROR:

     TWCR =   (1<<TWSTO)|(1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
     while(TWCR&_BV(TWSTO));  

     break;

    default:

      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Acknowledge on any new requests.
             (0<<TWWC);                                 //


  }


} 


void setup()
{
   DDRF = 0b00000000;
   DDRD = 0b01111000;
   PORTD= 0b00000000;
   DDRE = 0b00000000; // DDRE |= (1<<2);   be carefull with hwb, check if its connected to GND via 10k (prototypes!)
   DDRB = 0b11110110;
   DDRC = 0b11000000;
   OCR0A = 0;      //    start value for FAN  190 / 255  (-> p-channel so inverted)       0x00 is ON  0xFF is OFF
   TCCR0B  =  0b00000001;
   TCCR0A  =  0b10000001;            // 8bit dual slope 31khz
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
   wdt_enable(WDTO_1S);
   bllevel = 0;
   newbllevel = 31;
}

int main()
{
  uint8_t adcselect = 0;
  setup();

  while(1) {  



  wdt_reset();	
  if (isrtimer > 62500)   // routine for calculate fan speed - timer is 32khz, just to save ressources
  {rpm = fanspin * 15; // 2 signals each turn, double time but 60seconds        
  fanspin = 0;
  isrtimer = 0;

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
 
}} 

}
