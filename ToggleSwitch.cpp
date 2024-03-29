#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "debounce.h"

class ToggleSwitch
{
  private:

    volatile uint8_t* _inputRegister;
    uint8_t _inputPin;
    volatile uint8_t* _outputRegister;
    uint8_t _outputPin;

    uint8_t _inputPinMask;
    uint8_t _outputPinMask;

  public:

    ToggleSwitch() {}

    ToggleSwitch(
        volatile uint8_t* inputRegister,
        uint8_t inputPin,
        volatile uint8_t* outputRegister,
        uint8_t outputPin)
    {
        _inputRegister = inputRegister;
        _inputPin = inputPin,
        _outputRegister = outputRegister;
        _outputPin = outputPin;

        _inputPinMask = (1 << _inputPin);
        _outputPinMask = (1 << _outputPin);
    }

    bool Scan()
    {
        if (button_down(_inputPinMask))
        {
            *_outputRegister ^= _outputPinMask;

            return true;
        }

        return false;
    }

    void Set(bool isOn)
    {
        if (isOn)
        {
            *_outputRegister |= _outputPinMask;
        }
        else
        {
            *_outputRegister &= ~_outputPinMask;
        }

    }
};

class ExclusiveToggleSwitchGroup
{
  private:
    static const int _maxSwitches = 3;

    ToggleSwitch _switches[_maxSwitches];
    int _switchCount;

  public:
    ExclusiveToggleSwitchGroup() {}

    ExclusiveToggleSwitchGroup(
        ToggleSwitch toggleSwitch1,
        ToggleSwitch toggleSwitch2)
    {
        _switches[0] = toggleSwitch1;
        _switches[1] = toggleSwitch2;

        _switchCount = 2;
    }

    ExclusiveToggleSwitchGroup(
        ToggleSwitch toggleSwitch1,
        ToggleSwitch toggleSwitch2,
        ToggleSwitch toggleSwitch3)
    {
        _switches[0] = toggleSwitch1;
        _switches[1] = toggleSwitch2;
        _switches[2] = toggleSwitch3;

        _switchCount = 3;
    }

    bool Scan()
    {
        // set initial value as 'do-nothing' flag
        int switchIndex = _maxSwitches + 1;

        for (int i = 0; i < _switchCount; i++)
        {
            if (_switches[i].Scan())
            {
                switchIndex = i;
                break;
            }
        }

        if (switchIndex > _maxSwitches)
        {
            return false;
        }

        for (int i = 0; i < _switchCount; i++)
        {
            if (i == switchIndex)
            {
                _switches[i].Set(true);
                continue;
            }

            _switches[i].Set(false);
        }

        return true;
    }
};
