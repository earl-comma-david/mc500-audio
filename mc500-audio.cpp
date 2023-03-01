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
#include "Relay.cpp"

#define I2C_MESSAGE_LENGTH 3
#define I2C_ADDR 0x10
#define ATTENUATION_DATA_INDEX 0
#define SWITCH_DATA_UPPER_INDEX 1
#define SWITCH_DATA_LOWER_INDEX 2

#define WORD_MASK_INPUT_1  0b1000000000000000
#define WORD_MASK_INPUT_2  0b0100000000000000
#define WORD_MASK_INPUT_3  0b0010000000000000
#define WORD_MASK_OUTPUT_1 0b0000000001000000
#define WORD_MASK_OUTPUT_2 0b0000000000100000
#define WORD_MASK_OUTPUT_3 0b0000000000010000
#define WORD_MASK_SUB      0b0000000000001000
#define WORD_MASK_MONO     0b0000000000000100

#define RELAY_OUT_PIN_INPUT_1  PD2
#define RELAY_OUT_PIN_INPUT_2  PD3
#define RELAY_OUT_PIN_INPUT_3  PD5
#define RELAY_OUT_PIN_OUTPUT_1 PD6
#define RELAY_OUT_PIN_OUTPUT_2 PD7
#define RELAY_OUT_PIN_OUTPUT_3 PB0
#define RELAY_OUT_PIN_MONO     PB1

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

    uint8_t relayCount = 7;

    Relay relays[relayCount] =
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
            lastSwitchState = data;

            for (int i = 0; i < relayCount; i++)
            {
                relays[i].scan(data);
            }
        }
    }

    return 0;
}

