PROGRAM = program
BUILD_DIR = build
DEBUG_ENABLED = 1

SOURCE_S_LIST = \
src/startup_code.s \
src/instruction_set.s

SOURCE_C_LIST = \
src/main.c \
src/systick.c \
src/handlers.c

INCLUDE_C_LIST = \
-Iinc

OBJECTS_LIST = 
OBJECTS_LIST += $(addprefix $(BUILD_DIR)/, $(notdir $(SOURCE_S_LIST:.s=.o)))
OBJECTS_LIST += $(addprefix $(BUILD_DIR)/, $(notdir $(SOURCE_C_LIST:.c=.o)))
vpath %.s $(sort $(dir $(SOURCE_S_LIST)))
vpath %.c $(sort $(dir $(SOURCE_C_LIST)))

CC = arm-none-eabi-gcc.exe -c
AS = arm-none-eabi-gcc.exe -c -x assembler
LD = arm-none-eabi-ld.exe
HX = arm-none-eabi-objcopy.exe --output-target ihex

MCU = -mcpu=cortex-m3 -mthumb

FLAGS_DEBUG = -g -gdwarf-2 -fdebug-prefix-map==../

FLAGS_AS =
FLAGS_AS += $(MCU)
ifeq ($(DEBUG_ENABLED), 1)
FLAGS_AS += $(FLAGS_DEBUG)
endif
FLAGS_AS += -Wa,-mimplicit-it=thumb

FLAGS_CC =
FLAGS_CC += $(MCU)
ifeq ($(DEBUG_ENABLED), 1)
FLAGS_CC += $(FLAGS_DEBUG)
endif

FLAGS_CC += -MMD -MP
FLAGS_CC += $(INCLUDE_C_LIST)

LINKER_SCRIPT = linker_script.ld

all : $(BUILD_DIR)/$(PROGRAM).elf $(BUILD_DIR)/$(PROGRAM).hex

$(BUILD_DIR)/$(PROGRAM).hex : $(BUILD_DIR)/$(PROGRAM).elf
	$(HX) $(<) $(@)

$(BUILD_DIR)/$(PROGRAM).elf : $(OBJECTS_LIST) makefile
	$(LD) -T $(LINKER_SCRIPT) -o $(@) $(OBJECTS_LIST)

$(BUILD_DIR)/%.o : %.s makefile | $(BUILD_DIR)
	$(AS) $(FLAGS_AS) -o $(@) $(<)

$(BUILD_DIR)/%.o : %.c makefile | $(BUILD_DIR)
	$(CC) $(FLAGS_CC) -o $(@) $(<)
	
$(BUILD_DIR) :
	mkdir $(@)
	
clean :
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
