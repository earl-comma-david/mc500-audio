#include <avr/io.h>

volatile uint8_t* Register();
uint8_t PinId();

class Pin
{
    private:
        volatile uint8_t* _pinRegister;
        uint8_t _pin;

    public:
        Pin()
        {
        }

        Pin(
            volatile uint8_t* pinRegister,
            uint8_t pin)
        {
            _pinRegister = pinRegister;
            _pin = pin;
        }

        volatile uint8_t* Register()
        {
            return _pinRegister;
        }

        uint8_t PinId(void)
        {
            return _pin;
        }
};
