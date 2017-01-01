#
# Generic Makefile for STM32 projects, using a similar structure 
# that in STM32 Cube package.
#
# Copyright 2016 Victor Pecanins.
#
# Version 0.37
#
# Usage:  copy under a folder named GCC, side by side to SW4STM32 folder
#         under the respective folder under Projects
#
# - agrep only works in Linux
# - openocd only works in Linux
#
# Commands:
#

#	make			Compile project
#	make program	Burn program to STM32 using OpenOCD
#	make clean		Remove compiled files
#

# Build environment
SHELL := /bin/bash
MAKE  := make

# Path to GCC_ARM_NONE_EABI toolchain
#GCC_ARM_NONE_EABI_PATH := ~/workspace/STM32Toolchain/gcc-arm-none-eabi-5_2-2015q4/bin
#GCC_ARM_NONE_EABI_PATH := /home/vpecanins/Downloads/gcc-arm-none-eabi-5_4-2016q3/bin
GCC_ARM_NONE_EABI_PATH := /c/arm-toolchain/bin
CC      := $(GCC_ARM_NONE_EABI_PATH)/arm-none-eabi-gcc
AR      := $(GCC_ARM_NONE_EABI_PATH)/arm-none-eabi-ar
OBJCOPY := $(GCC_ARM_NONE_EABI_PATH)/arm-none-eabi-objcopy
OBJDUMP := $(GCC_ARM_NONE_EABI_PATH)/arm-none-eabi-objdump
SIZE    := $(GCC_ARM_NONE_EABI_PATH)/arm-none-eabi-size

## Everything from here down depends from the magic variables:
PROJ_LVL       := $(shell pwd | rev | sed -e "s/\//\n/g"  | rev | sed -n "/Project/{=; q;}")
STM32CUBE_ROOT := $(shell for i in $$(seq 1 $(PROJ_LVL)); do echo "../"; done | tr -d '\n')
PROJ_NAME      := $(shell pwd | rev | cut -d "/" -f 2  | rev)
PROJ_PLATFORM  := $(shell pwd | rev | cut -d "/" -f $$(( $(PROJ_LVL) - 1 )) | rev )  
STM32ANNNAA    := $(shell echo $(PROJ_PLATFORM) | cut -d "-" -f 1 | sed 's/STM327/STM32F7/g')
STM32ANNN      := $(shell echo $(STM32ANNNAA) | cut -c -9)
STM32AN        := $(shell echo $(STM32ANNNAA) | cut -c -7)
STM32FAMILY    := $(shell echo $(STM32ANNNAA) | cut -c 6-7)
BOARDTYPE      := $(shell echo $(PROJ_PLATFORM) | cut -d "-" -f 2)
SW4STM32_NAME  := $(PROJ_PLATFORM)
SRC_PATH       := $(shell echo $(STM32CUBE_ROOT)Projects/$(PROJ_PLATFORM)/$(PROJ_NAME)/Src | tr -d ' ')

## Catch-all alternatives
## Comment to ST employees: Please regularize names in STM Cube packages.
BSP_NAME_0     := $(STM32AN)xx_$(BOARDTYPE)
HAL_NAME       := $(STM32AN)xx_HAL_Driver
CMSIS_NAME     := $(STM32AN)xx


## Approximate guessing with agrep
#BSP_NAME_REAL  := $(shell ls "$(STM32CUBE_ROOT)Drivers/BSP/" | agrep -5 $(BSP_NAME))
#HAL_NAME_REAL  := $(shell ls "$(STM32CUBE_ROOT)Drivers/" | agrep -5 $(HAL_NAME))
#CMSIS_NAME_REAL:= $(shell ls "$(STM32CUBE_ROOT)Drivers/CMSIS/Device/ST/" | agrep -5 $(CMSIS_NAME))
BSP_NAME_REAL  := STM32746G-Discovery
HAL_NAME_REAL  := $(HAL_NAME)
CMSIS_NAME_REAL:= $(CMSIS_NAME)

## Getting the appropriate asm file is *very* tricky.
ASM_PATH       := $(STM32CUBE_ROOT)Drivers/CMSIS/Device/ST/$(CMSIS_NAME_REAL)/Source/Templates/gcc
WC_NAMES       := $(shell ls $(ASM_PATH) | grep "^.*\.s" | sed 's/startup_//g' | sed 's/.s//g' | sed 's/x/./g')
#WC_MATCH       := $(shell for a in $(WC_NAMES); do if [ ! -z $$(echo $(STM32ANNNAA) | grep -i $$a) ]; then echo $$a; fi; done) 
WC_MATCH       := $(shell echo $(WC_NAMES) | tr ' ' '\n' | grep -i $(STM32ANNN) ) # FOR STM32F7
STM32ANNNXN    := $(shell echo $(WC_MATCH) | tr a-z A-Z | tr '.' 'x')

# OPENOCD Path and arguments
OPENOCD_DIR    := ~/workspace/openocd-stm32f7/src
OPENOCD_SCRIPTS:= board/stm32f7discovery.cfg ./stm_flash.cfg
OPENOCD_COMMAND:= "stm_flash $(PROJ_NAME).bin"

# LDscript for linking for STM32, from SW4STM32
#LDSCRIPT := $(shell find ../SW4STM32/$(SW4STM32_NAME) -name *.ld)
#LDSCRIPT := $(shell find ../SW4STM32 -name *.ld)
LDSCRIPT := $(shell find -name *.ld)

# Project include directories
INCLUDEDIRS =  ../Inc
INCLUDEDIRS += $(STM32CUBE_ROOT)Drivers/BSP/$(BSP_NAME_REAL)
INCLUDEDIRS += $(wildcard $(STM32CUBE_ROOT)Drivers/BSP/Components/*)
INCLUDEDIRS += $(STM32CUBE_ROOT)Drivers/$(HAL_NAME_REAL)/Inc
INCLUDEDIRS += $(STM32CUBE_ROOT)Drivers/CMSIS/Include
INCLUDEDIRS += $(STM32CUBE_ROOT)Drivers/CMSIS/Device/ST/$(CMSIS_NAME_REAL)/Include

# Project source files
SOURCES =  $(shell find $(SRC_PATH) -iname "system_stm32$(STM32FAMILY)xx.c")
SOURCES += $(shell find $(SRC_PATH) -iname "stm32$(STM32FAMILY)xx_it.c")
SOURCES += $(shell find $(SRC_PATH) -iname "stm32$(STM32FAMILY)xx_hal_msp.c")
SOURCES += $(shell find $(SRC_PATH) -iname "cube_hal_$(STM32FAMILY).c")
SOURCES += $(shell find $(SRC_PATH) -not -iname "cube_hal_*" -and -not -iname "*stm32*" -and -name "*.c")

#SOURCES += $(shell find ../SW4STM32/$(SW4STM32_NAME) -name syscalls.c)
SOURCES += $(shell find $(STM32CUBE_ROOT)Drivers/BSP/$(BSP_NAME_REAL)/ -name "*.c")
SOURCES += $(shell find $(STM32CUBE_ROOT)Drivers/BSP/Components/  -name "*.c")
SOURCES += $(shell find $(STM32CUBE_ROOT)Drivers/$(HAL_NAME_REAL)/Src -name "*.c")
SOURCES += $(shell find $(STM32CUBE_ROOT)Drivers/CMSIS/DSP_Lib/Source -name "*.c")

# Add assembly startup template
SOURCE_AS := $(STM32CUBE_ROOT)Drivers/CMSIS/Device/ST/$(CMSIS_NAME_REAL)/Source/Templates/gcc/startup_$(shell echo $(STM32ANNNXN) | tr A-Z a-z).s

# Generate target object files from sources
OBJ = $(SOURCES:.c=.o)
OBJ_AS = $(SOURCE_AS:.s=.o)

# Static libraries
CLIBS += -Wl,--start-group -lc -lm -Wl,--end-group


# Compiler flags common for all STM32
# Add: -std=gnu99 if you get the error: expected '=', ',', ';', 'asm' or '__attribute__' before 'asm'
CFLAGS  = -Wall -g3 -Os -Werror-implicit-function-declaration
CFLAGS += -ffunction-sections -fmessage-length=0 -mfloat-abi=soft 
CFLAGS += -Wl,-gc-sections -Wl,-Map=$(PROJ_NAME).map -specs=nano.specs

# Compiler define (Guidance for HAL and BSP headers ifdefs &c.)
CFLAGS += -D$(STM32ANNNXN)

# Compiler flags (SPECIFIC STM32F0)
#CFLAGS += -mthumb -mcpu=cortex-m0plus -specs=nano.specs 

# Compiler flags (SPECIFIC STM32F7)
CFLAGS +=  -std=gnu99 -mlittle-endian -mcpu=cortex-m7 -mthumb -DARM_MATH_CM7 -nostartfiles
CFLAGS +=  -fdata-sections -ffunction-sections 

# Compiler define (additional, needed for some projects)
CFLAGS += -DUSE_USB_HS

# Include directories & static libs (normally this is the longest part)
CFLAGS +=  $(addprefix -I,$(INCLUDEDIRS)) $(CLIBS)

# Commands for recipes
COMPILE_C    = $(CC) -c $< -o $@ $(CFLAGS)
COMPILE_S    = $(CC) -c $< -o $@ $(CFLAGS)
COMPILE_LINK = $(CC) -Os $(CFLAGS) $^ -o $@ -T $(LDSCRIPT) -MD
BUILD_PRINT  = @echo -e "\e[1;32mBuilding $@\e[0m"

.PHONY: all

all: $(PROJ_NAME).elf

magic:
	@echo "Magic-generated information:"    
	@echo "Root:         $(STM32CUBE_ROOT)"
	@echo "Project Name: $(PROJ_NAME)"
	@echo "Platform:     $(PROJ_PLATFORM)"
	@echo "Family:       $(STM32FAMILY)"
	@echo "STM32F0:      $(STM32AN)"
	@echo "STM32F072:    $(STM32ANNN)"
	@echo "STM32F072RB:  $(STM32ANNNAA)"
	@echo "STM32F072xB:  $(STM32ANNNXN)"
	@echo "Board type:   $(BOARDTYPE)"
	@echo "BSP Name:     $(BSP_NAME)"
	@echo "HAL Name:     $(HAL_NAME)"
	@echo "CMSIS Name:   $(CMSIS_NAME)"
	
info:
	@echo "Build info:"
	@echo "Actual BSP:   $(BSP_NAME_REAL)"
	@echo "Actual HAL:   $(HAL_NAME_REAL)"
	@echo "Actual CMSIS: $(CMSIS_NAME_REAL)"
	@echo $(SRC_PATH)
	@echo "Startup ASM:  $(SOURCE_AS)"
	@echo "Linker Script $(LDSCRIPT)"

info-sources:
	@echo "Source files:"
	@echo $(SOURCES) | sed 's/ /\n/g'
	@echo $(SOURCE_AS) | sed 's/ /\n/g'

info-includes:
	@echo "Include directories:"
	@echo $(INCLUDEDIRS) | sed 's/ /\n/g'

%.o: %.c
	@echo
	$(BUILD_PRINT)
	@echo "$(COMPILE_C)" | fmt
	@$(COMPILE_C)
#	@aplay -r 10000 $@ # This is fun

$(OBJ_AS): $(SOURCE_AS)
	@echo
	$(BUILD_PRINT)
	@echo "$(COMPILE_S)" | fmt
	@$(COMPILE_S)

$(PROJ_NAME).elf: $(OBJ) $(OBJ_AS)
	@echo
	$(BUILD_PRINT)
	@echo "$(COMPILE_LINK)" | fmt
	@$(COMPILE_LINK)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf 

size:
	@$(SIZE) $(PROJ_NAME).elf $(OBJ) $(OBJ_AS) | sort -k4 -n
    
program: $(PROJ_NAME).elf stm_flash.cfg
	$(OPENOCD_DIR)/openocd $(addprefix -f,$(OPENOCD_SCRIPTS)) -c $(OPENOCD_COMMAND) -c shutdown

debug: $(PROJ_NAME).elf stm_flash.cfg
	$(OPENOCD_DIR)/openocd $(addprefix -f,$(OPENOCD_SCRIPTS)) -c $(OPENOCD_COMMAND)
	
program-mbed: $(PROJ_NAME).elf
	cp $(PROJ_NAME).bin /d/

clean:
	@rm -rf $(OBJ)
	@echo Removing objects...
	rm -rf $(OBJ_AS)
	rm -rf $(PROJ_NAME).elf
	rm -rf $(PROJ_NAME).hex
	rm -rf $(PROJ_NAME).bin
	rm -rf $(PROJ_NAME).map
	rm -rf $(PROJ_NAME).lst 
	rm -rf $(PROJ_NAME).d
	rm -rf stm_flash.cfg
	
stm_flash.cfg:
	@echo init > stm_flash.cfg
	@echo >> stm_flash.cfg
	@echo "proc stm_flash {IMGFILE} {" >> stm_flash.cfg
	@echo "  reset halt" >> stm_flash.cfg
	@echo "  sleep 100" >> stm_flash.cfg
	@echo "  wait_halt 2" >> stm_flash.cfg
	@echo "  flash write_image erase \$$IMGFILE 0x08000000" >> stm_flash.cfg
	@echo "  sleep 100" >> stm_flash.cfg
	@echo "  verify_image \$$IMGFILE 0x08000000" >> stm_flash.cfg
	@echo "  sleep 100" >> stm_flash.cfg
	@echo "  reset run" >> stm_flash.cfg
	@echo "}" >> stm_flash.cfg

