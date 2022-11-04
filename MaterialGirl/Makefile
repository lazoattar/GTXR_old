TARGET = main

BUILD_DIR = build

C_SOURCES = $(shell find ./src -name "*.c")

PREFIX = arm-none-eabi
CC = $(PREFIX)-gcc
AS = $(PREFIX)-as
CP = $(PREFIX)-objcopy
LD = $(PREFIX)-gcc
SZ = $(PREFIX)-size
GDB = $(PREFIX)-gdb

BIN = $(CP) -O binary

CPU = -mcpu=cortex-m7
MCU = $(CPU) -g --specs=nosys.specs -Wl,-Map="MaterialGirl.map" -Wl,--gc-sections --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group

DIR = Drivers

C_INCLUDES = -I.
C_INCLUDES += -I./${DIR}/CMSIS/Device/ST/STM32H7xx/Include
C_INCLUDES += -I./${DIR}/CMSIS/Include
C_INCLUDES += -I./${DIR}/CMSIS/DSP/Include
C_INCLUDES += -I./${DIR}/STM32H7xx_HAL_Driver/Inc
C_INCLUDES += -I./include

CFLAGS = $(MCU) $(C_INCLUDES)

LDSCRIPT = stm32.ld
LDFLAGS = -T $(LDSCRIPT) -static -L./${DIR}/CMSIS/DSP/Lib/GCC -larm_cortexM7lfdp_math

all: $(BUILD_DIR)/$(TARGET).bin

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@
$(BUILD_DIR)/%.o: %.S | $(BUILD_DIR)
	$(AS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | $(BUILD_DIR)
	$(LD) -o $@ $(OBJECTS) $(LDFLAGS) $(CFLAGS)
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
