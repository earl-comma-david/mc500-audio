#include <avr/io.h>
#include "Pin.cpp"

void Scan();

class Relay
{
    private:
        uint16_t _input_mask;
        volatile uint8_t* _outputRegister;
        uint8_t _outputPin;

    public:

        Relay()
        {
        }

        Relay(
            uint16_t input_mask,
            Pin outputPin)
                : Relay(input_mask, outputPin.Register(), outputPin.PinId())
        {
        }

        Relay(
            uint16_t input_mask,
            volatile uint8_t* outputRegister,
            uint8_t outputPin)
        {
            _input_mask = input_mask;
            _outputRegister = outputRegister;
            _outputPin = outputPin;
        }

        void Scan(uint16_t input)
        {
            if (input & _input_mask)
            {
                *_outputRegister |= (1<<_outputPin);
            }
            else
            {
               *_outputRegister &= ~(1<<_outputPin);
            }
        }
};
