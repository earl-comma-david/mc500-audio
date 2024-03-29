#define I2C_MESSAGE_LENGTH 3
#define I2C_ADDR 0x10
#define ATTENUATION_WORD_INDEX 0
#define SWITCH_WORD_UPPER_INDEX 1
#define SWITCH_WORD_LOWER_INDEX 2

#define WORD_MASK_INPUT_1  0b0100000000000000
#define WORD_MASK_INPUT_2  0b0010000000000000
#define WORD_MASK_INPUT_3  0b0001000000000000
#define WORD_MASK_INPUT_4  0b0000100000000000
#define WORD_MASK_MONO     0b0000010000000000
#define WORD_MASK_OUTPUT_1 0b0000000001000000
#define WORD_MASK_OUTPUT_2 0b0000000000100000
#define WORD_MASK_OUTPUT_3 0b0000000000010000
#define WORD_MASK_SUB      0b0000000000001000

#define RELAY_OUT_PIN_INPUT_1  PD2
#define RELAY_OUT_PIN_INPUT_2  PD3
#define RELAY_OUT_PIN_INPUT_3  PD4
//#define RELAY_OUT_PIN_INPUT_4  PD5
#define RELAY_OUT_PIN_MONO     PB1
#define RELAY_OUT_PIN_OUTPUT_1 PD6
#define RELAY_OUT_PIN_OUTPUT_2 PD7
#define RELAY_OUT_PIN_OUTPUT_3 PB0
// TODO:
#define RELAY_OUT_PIN_SUB      PB1
// TODO: rewire board with updated pins:
//#define RELAY_OUT_PIN_INPUT_3  PD4
//#define RELAY_OUT_PIN_MONO     PD5
//#define RELAY_OUT_PIN_OUTPUT_1 PD6
//#define RELAY_OUT_PIN_OUTPUT_2 PD7
//#define RELAY_OUT_PIN_OUTPUT_3 PB0
//#define RELAY_OUT_PIN_SUB      PB1
