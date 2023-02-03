#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ShiftRegister.cpp"
#include "debounce.h"
#include "debounce.cpp"
#include "twi-slave.h"
#include "twi-slave.c"

volatile uint8_t _i2cIndex;
volatile uint8_t _data;
volatile uint8_t _dataRight;

#define I2C_ADDR 0x10

void I2C_received(uint8_t received_data)
{
  if (_i2cIndex++ % 2 == 0)
  {
    _data = received_data;
  }
  else
  {
    _dataRight = received_data;
  }
}

void I2C_requested()
{
  I2C_transmitByte(_data);
}
 
int main (void)
{
  DDRB = 0xFF;
  DDRD = 0xFF;
  PORTB = 0x00;
  PORTD = 0x00;

  ShiftRegister shiftRegister = ShiftRegister();
  //DDRB |= (1<<PB5);

    //DDRB |= (1<<PB0);
    //DDRD = 0xff;

    I2C_setCallbacks(I2C_received, I2C_requested);
    I2C_init(I2C_ADDR);
    _data = 0x00;

    uint8_t lastData = 0xff;

    uint8_t byte = 0;
    while(1)
    {
        //PORTB |= 1<<PB5;
        //_delay_ms(100);
        //PORTB &= ~(1<<PB5);
        //_delay_ms(50);
        //PORTB |= 1<<PB5;
        //_delay_ms(100);
        //PORTB &= ~(1<<PB5);

        if (_data != lastData)
        {
          shiftRegister.put(_data);
          //shiftRegister.put(255 - byte++);
          lastData = _data;
        }

        //PORTD |= 1<<PD2;
        //_delay_ms(1000);
        //PORTD &= ~(1<<PD2);
        //_delay_ms(1000);
        //PORTD |= 1<<PD3;
        //_delay_ms(1000);
        //PORTD &= ~(1<<PD3);
        //_delay_ms(1000);
        //PORTD |= 1<<PD5;
        //_delay_ms(1000);
        //PORTD &= ~(1<<PD5);
        //_delay_ms(1000);
        //PORTD |= 1<<PD6;
        //_delay_ms(1000);
        //PORTD &= ~(1<<PD6);
        //_delay_ms(1000);
        //PORTD |= 1<<PD7;
        //_delay_ms(1000);
        //PORTD &= ~(1<<PD7);
        //_delay_ms(1000);
        //PORTB |= 1<<PB0;
        //_delay_ms(1000);
        //PORTB &= ~(1<<PB0);
        //_delay_ms(1000);
        //PORTB |= 1<<PB1;
        //_delay_ms(1000);
        //PORTB &= ~(1<<PB1);
        //_delay_ms(1000);

        //PORTD = _data;

        //_delay_ms(100);
    }

    return 0;
}

