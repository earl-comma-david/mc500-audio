#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include "twi-slave.h"
#include "twi-slave.c"
#include "Relay.cpp"
#include "ShiftRegister.cpp"

#include "config.h"

volatile uint8_t _i2cIndex = 0;

// TODO: master reads its initial state from the slave
// message: { attenuationWord, inputSelectWord, outputSelectWord }
volatile uint8_t _lastI2cMessage[I2C_MESSAGE_LENGTH] = { 127, 0b01100000, 0b01100000 };

void I2C_received(uint8_t received_data)
{
    _lastI2cMessage[_i2cIndex++] = received_data;
    if (_i2cIndex == I2C_MESSAGE_LENGTH)
    {
        _i2cIndex = 0;
    }
}

void I2C_requested()
{
  // TODO:
  //I2C_transmitByte(_data);
}

void init_io_pins()
{
    // pullups
    PORTC |= (1<<PC5) | (1<<PC4);

    // outputs
    // TODO: config consts
    DDRB |= (1<<PB4);
    DDRB |= (1<<PB3);
    DDRB |= (1<<PB2);
    DDRB |= (1<<PB1);
    DDRB |= (1<<PB0);
    DDRD |= (1<<PD7);
    DDRD |= (1<<PD6);
    DDRD |= (1<<PD5);
    DDRD |= (1<<PD4);
    DDRD |= (1<<PD3);
    DDRD |= (1<<PD2);

    // inputs
    DDRB &= ~(1 << PINB5);
}

void init(void)
{
    I2C_setCallbacks(I2C_received, I2C_requested);
    I2C_init(I2C_ADDR);

    init_io_pins();

    UCSR0B = 0;
}
 
int main (void)
{
    init();

    ShiftRegister switchShiftRegister = ShiftRegister(&PORTD, PD2, PD3, PD5);
    ShiftRegister attenuationShiftRegisterMain = ShiftRegister(&PORTD, PD6, &PORTD, PD7, &PORTB, PB0);
    ShiftRegister attenuationShiftRegisterHP = ShiftRegister(&PORTB, PB2, PB3, PB4);
    uint16_t lastSwitchState = 0xff;
    uint8_t lastAttenuationStateMain = 0x00;
    uint8_t lastAttenuationStateHP = 0x00;

    while (1)
    {
        if (_lastI2cMessage[ATTENUATION_WORD_INDEX] != lastAttenuationStateMain)
        {
            lastAttenuationStateMain = _lastI2cMessage[ATTENUATION_WORD_INDEX];
            attenuationShiftRegisterMain.shiftOut((uint8_t) (127 - lastAttenuationStateMain));
        }

        uint16_t ioSelectWord =
            (_lastI2cMessage[SWITCH_WORD_UPPER_INDEX] << 8)
            | _lastI2cMessage[SWITCH_WORD_LOWER_INDEX];

        if (ioSelectWord != lastSwitchState)
        {
            switchShiftRegister.shiftOut(ioSelectWord);

            lastSwitchState = ioSelectWord;
        }
    }

    return 0;
}

