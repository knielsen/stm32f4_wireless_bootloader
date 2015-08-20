TARGET=wireless_bootloader

OBJS = $(TARGET).o dbg.o led.o

STM_DIR=/home/knielsen/devel/study/stm32f4/STM32F4-Discovery_FW_V1.1.0
STM_SRC = $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src
vpath %.c $(STM_SRC)
STM_OBJS = system_stm32f4xx.o
STM_OBJS  += stm32f4xx_rcc.o
STM_OBJS  += stm32f4xx_gpio.o
STM_OBJS  += stm32f4xx_usart.o
STM_OBJS  += stm32f4xx_flash.o
STM_OBJS  += misc.o

INC_DIRS  = $(STM_DIR)/Utilities/STM32F4-Discovery
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Include
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/ST/STM32F4xx/Include
INC_DIRS += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INC_DIRS += .
INC = $(addprefix -I,$(INC_DIRS))

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy


STARTUP_OBJ=my_startup_stm32f4xx.o
STARTUP_SRC=my_startup_stm32f4xx.s
LINKSCRIPT=$(TARGET).ld

ARCH_FLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections

CFLAGS=-ggdb -Os -std=c99 -Wall -Wextra -Warray-bounds $(ARCH_FLAGS) $(INC) -DUSE_STDPERIPH_DRIVER
LDFLAGS=-Wl,--gc-sections -lm


.PHONY: all flash clean tty cat

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STM_OBJS) $(STARTUP_OBJ) $(LINKSCRIPT)
	$(LD) $(ARCH_FLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP_OBJ) $(OBJS) $(STM_OBJS) $(LDFLAGS)

$(TARGET).o: $(TARGET).c

$(STARTUP_OBJ): $(STARTUP_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

dbg.o: dbg.h

leds.c: leds.h

wireless_bootloader.c: dbg.h led.h nrf24l01p.h

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x8000000

clean:
	rm -f $(OBJS) $(STM_OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP_OBJ)

tty:
	stty -F/dev/ttyACM1 raw -echo -hup cs8 -parenb -cstopb 115200

cat:
	cat /dev/ttyACM1
