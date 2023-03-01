#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ShiftRegister.cpp"
#include "twi-slave.h"
#include "twi-slave.c"
#include "Relay.cpp"

#include "config.h"

volatile uint8_t _i2cIndex;
// TODO: master reads its initial state from the slave
volatile uint8_t _data[I2C_MESSAGE_LENGTH] = { 127, 0b10000000, 0b01000000 };

void I2C_received(uint8_t received_data)
{
    _data[_i2cIndex++] = received_data;
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
    DDRD |= (1<<PD3);
    DDRD |= (1<<PD2);
}

void init(void)
{
    I2C_setCallbacks(I2C_received, I2C_requested);
    I2C_init(I2C_ADDR);

    init_io_pins();
}
 
int main (void)
{
    init();

    Relay relays[7] =
    {
        Relay(WORD_MASK_INPUT_1, &PORTD, RELAY_OUT_PIN_INPUT_1),
        Relay(WORD_MASK_INPUT_2, &PORTD, RELAY_OUT_PIN_INPUT_2),
        Relay(WORD_MASK_INPUT_3, &PORTD, RELAY_OUT_PIN_INPUT_3),
        Relay(WORD_MASK_OUTPUT_1, &PORTD, RELAY_OUT_PIN_OUTPUT_1),
        Relay(WORD_MASK_OUTPUT_2, &PORTD, RELAY_OUT_PIN_OUTPUT_2),
        Relay(WORD_MASK_OUTPUT_3, &PORTB, RELAY_OUT_PIN_OUTPUT_3),
        Relay(WORD_MASK_MONO, &PORTB, RELAY_OUT_PIN_MONO)
    };

    ShiftRegister mainAttenuationShiftRegister = ShiftRegister();
    uint8_t lastAttenuationState = 0x00;
    uint16_t lastSwitchState = 0xff;
    uint8_t byte = 0;

    while(1)
    {
        if (_data[ATTENUATION_DATA_INDEX] != lastAttenuationState)
        {
            // TODO: reevaluate where the best place for this negation is:
            lastAttenuationState = ~_data[ATTENUATION_DATA_INDEX];
            mainAttenuationShiftRegister.put(lastAttenuationState);
        }

        uint16_t data = (_data[SWITCH_DATA_UPPER_INDEX] << 8) | _data[SWITCH_DATA_LOWER_INDEX];

        if (data != lastSwitchState)
        {
            for (Relay relay : relays)
            {
                relay.Scan(data);
            }

            lastSwitchState = data;
        }
    }

    return 0;
}

