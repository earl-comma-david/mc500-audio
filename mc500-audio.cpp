#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ShiftRegister.cpp"
//#include "twi-slave.h"
//#include "twi-slave.c"
#include "Relay.cpp"

#include "config.h"

volatile uint8_t _i2cIndex = 0;

// TODO: master reads its initial state from the slave
// message: { attenuationWord, inputSelectWord, outputSelectWord }
volatile uint8_t _lastI2cMessage[I2C_MESSAGE_LENGTH] = { 127, 0b01111000, 0b01110000 };
//volatile uint8_t _lastI2cMessage[I2C_MESSAGE_LENGTH] = { 127, 0b01000000, 0b01000000 };

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
    // outputs
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
}

void init(void)
{
    //I2C_setCallbacks(I2C_received, I2C_requested);
    //I2C_init(I2C_ADDR);

    init_io_pins();

    UCSR0B = 0;

    TCNT1 = 65500; // for 1 sec at 16 MHz	
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (1<<CS12);  // Timer mode with 1024 prescler
	TIMSK1 = (1 << TOIE1) ;
}
 
int main (void)
{
    init();

    Relay relays[8] =
    {
        Relay(WORD_MASK_INPUT_1, &PORTD, RELAY_OUT_PIN_INPUT_1),
        Relay(WORD_MASK_INPUT_2, &PORTD, RELAY_OUT_PIN_INPUT_2),
        Relay(WORD_MASK_INPUT_3, &PORTD, RELAY_OUT_PIN_INPUT_3),
        //Relay(WORD_MASK_INPUT_4, &PORTD, RELAY_OUT_PIN_INPUT_4),
        Relay(WORD_MASK_OUTPUT_1, &PORTD, RELAY_OUT_PIN_OUTPUT_1),
        Relay(WORD_MASK_OUTPUT_2, &PORTD, RELAY_OUT_PIN_OUTPUT_2),
        Relay(WORD_MASK_OUTPUT_3, &PORTB, RELAY_OUT_PIN_OUTPUT_3),
        Relay(WORD_MASK_SUB, &PORTB, RELAY_OUT_PIN_SUB),
        Relay(WORD_MASK_MONO, &PORTB, RELAY_OUT_PIN_MONO)
    };

    ShiftRegister attenuationShiftRegisterMain = ShiftRegister();
    ShiftRegister attenuationShiftRegisterHP = ShiftRegister();
    uint8_t lastAttenuationStateMain = 0x00;
    uint8_t lastAttenuationStateHP = 0x00;
    uint16_t lastSwitchState = 0xff;

    uint16_t states[9] =
    {
        0b0100000000000000,
        0b0010000000000000,
        0b0001000000000000,
        0b0000100000000000,
        0b0000010000000000,
        0b0000000001000000,
        0b0000000000100000,
        0b0000000000010000,
        0b0000000000001000
    };
    uint8_t i = 0;
    while (1)
    {
        //_delay_ms(1000);

        if (_lastI2cMessage[ATTENUATION_WORD_INDEX] != lastAttenuationStateMain)
        {
            lastAttenuationStateMain = _lastI2cMessage[ATTENUATION_WORD_INDEX];
            attenuationShiftRegisterMain.put(127 - lastAttenuationStateMain);
        }

        //uint16_t ioSelectWord = states[i++];
        //if (i == 9)
        //    i = 0;

        uint16_t ioSelectWord =
            (_lastI2cMessage[SWITCH_WORD_UPPER_INDEX] << 8)
            | _lastI2cMessage[SWITCH_WORD_LOWER_INDEX];

        if (ioSelectWord != lastSwitchState)
        {
            for (Relay relay : relays)
            {
                relay.Scan(ioSelectWord);
            }

            lastSwitchState = ioSelectWord;
        }
    }

    return 0;
}

