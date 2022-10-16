TARGET = main

BUILD_DIR = build

C_SOURCES = $(shell find . -name "*.c")

PREFIX = arm-none-eabi
CC = $(PREFIX)-gcc
AS = $(PREFIX)-as
CP = $(PREFIX)-objcopy
LD = $(PREFIX)-ld.bfd
SZ = $(PREFIX)-size
GDB = $(PREFIX)-gdb

BIN = $(CP) -O binary

CPU = -mcpu=cortex-m7
MCU = $(CPU) -mthumb -g

DIR = STM32F767_Drivers

C_INCLUDES = -I.
C_INCLUDES += -I./${DIR}/CMSIS/Device/ST/STM32F7xx/Include
C_INCLUDES += -I./${DIR}/CMSIS/Include
C_INCLUDES += -I./include

CFLAGS = $(MCU) $(C_INCLUDES)

LDSCRIPT = stm32.ld
LDFLAGS = -T $(LDSCRIPT)

all: $(BUILD_DIR)/$(TARGET).bin

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.S | $(BUILD_DIR)
	$(AS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | $(BUILD_DIR)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR $(BUILD_DIR)

flash:
	st-flash --reset write ./$(BUILD_DIR)/$(TARGET).bin 0x08000000

openocd:
	openocd -f ../utils/openocd.cfg

telnet:
	telnet localhost 4444

gdb: $(BUILD_DIR)/$(TARGET).elf
	$(GDB) --eval-command="target remote localhost:3333" $<
