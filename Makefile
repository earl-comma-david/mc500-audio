TARGET = mc500-audio
SRCS=mc500-audio.cpp

MCU   = atmega328p
F_CPU = 16000000UL  
BAUD  = 9600UL
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. 

CC = avr-g++
OBJCOPY = avr-objcopy
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
# CFLAGS += -funsigned-char -funsigned-bitfields 
# CFLAGS += -fpack-struct -fshort-enums 
# CFLAGS += -ffunction-sections -fdata-sections 


all:
	${CC} ${CFLAGS} -o ${TARGET}.o ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.o ${TARGET}.hex

flash:
	avrdude -c arduino -p ${MCU} -P /dev/cu.usbserial-1 -b57600 -U flash:w:${TARGET}.hex:i
	#avrdude -p ${MCU} -c usbtiny -U flash:w:${TARGET}.hex:i -F

clean:
	rm -f *.o *.hex
