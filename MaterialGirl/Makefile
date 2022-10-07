PROJECT=main

CC = arm-none-eabi-gcc
SF = st-flash
CFLAGS = -c -fno-common -mcpu=cortex-m7 -mthumb
CFLAGS_DEBUG = $(CFLAGS) -O0 -g

LD = arm-none-eabi-ld
LDFLAGS = -v -T stm32.ld -n
OBJCOPY = arm-none-eabi-objcopy

DIR = STM32F767_Drivers

INCLUDE  = -I.
#INCLUDE += -I${HOME}/.config/stm32/${DIR}/CMSIS/Device/ST/STM32F7xx/Include
#INCLUDE += -I${HOME}/.config/stm32/${DIR}/CMSIS/Include
INCLUDE += -I./${DIR}/CMSIS/Device/ST/STM32F7xx/Include
INCLUDE += -I./${DIR}/CMSIS/Include
INCLUDE += -I./include

SRCS = ./src/main.c

OBJS = $(SRCS:.c=.o)

debug:$(PROJECT).bin
$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -S -O binary $(PROJECT).elf $(PROJECT).bin

$(PROJECT).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $(PROJECT).elf $(OBJS)

$(OBJS):$(SRCS)
	$(CC) $(INCLUDE) $(CFLAGS_DEBUG) -o $(OBJS) $(SRCS)

clean:
	rm -f *.o *.elf *.bin *~

flash:
	$(SF) write $(PROJECT).bin 0x8000000
