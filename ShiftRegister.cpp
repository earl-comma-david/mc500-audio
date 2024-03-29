#include <avr/io.h>
#include <util/delay.h>

void put(uint8_t);
void putBit(uint8_t);
void latch();
void cycleClock();
void longDelay(uint16_t);
void setup();
uint8_t rotateLeft(uint8_t);
uint8_t rotateRight(uint8_t);

class ShiftRegister 
{
  private:
    volatile uint8_t* _outputPortSer;
    volatile uint8_t* _outputPortLatch;
    volatile uint8_t* _outputPortClk;
    uint8_t _pinSer;
    uint8_t _pinLatch;
    uint8_t _pinClk;

  public:

    ShiftRegister(volatile uint8_t* outputPort, uint8_t pinSer, uint8_t pinLatch, uint8_t pinClk)
    {
        _outputPortSer = outputPort;
        _pinSer = pinSer;
        _pinLatch = pinLatch;
        _outputPortLatch = outputPort;
        _pinClk = pinClk;
        _outputPortClk = outputPort;
    }

    ShiftRegister(
        volatile uint8_t* outputPortSer,
        uint8_t pinSer,
        volatile uint8_t* outputPortLatch,
        uint8_t pinLatch,
        volatile uint8_t* outputPortClk,
        uint8_t pinClk)
    {
        _outputPortSer = outputPortSer;
        _pinSer = pinSer;
        _outputPortLatch = outputPortLatch;
        _pinLatch = pinLatch;
        _outputPortClk = outputPortClk;
        _pinClk = pinClk;
    }

    uint8_t rotateLeft(uint8_t toRotate) {
        uint8_t carry = (toRotate & 0b10000000) == 0b10000000;
        return (toRotate << 1 | carry);
    }

    uint8_t rotateRight(uint8_t toRotate) {
        uint8_t carry = ( toRotate & 1 ) ? 0b10000000 : 0;
        return (toRotate >> 1 | carry);
    }

    void shiftOut(uint16_t toPut) {
        uint8_t i;
        for(i = 0; i < 16; i++) {
            putBit(toPut & 1);
            toPut >>= 1;
        }

        latch();
    }

    void shiftOut(uint8_t toPut) {
        uint8_t i;
        for(i = 0; i < 8; i++) {
            putBit(toPut & 1);
            toPut >>= 1;
        }

        latch();
    }


    void putBit(uint8_t bit) {
        if(bit == 0)
            *_outputPortSer &= ~(1<<_pinSer);
        else
            *_outputPortSer |= (1<<_pinSer);

        cycleClock();
    }

    void latch() {
        *_outputPortLatch |= (1<<_pinLatch);
        *_outputPortLatch &= ~(1<<_pinLatch);
    }

    void cycleClock() {
        *_outputPortClk |= (1<<_pinClk);
        _delay_us(5);
        *_outputPortClk &= ~(1<<_pinClk);
        _delay_us(5);
    }
};
