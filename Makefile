# Put your STM32F4 library code directory here
CAN_DIR=CanNode
I2C_DIR=I2C
SRC_DIR=Src
INC_DIR=Inc
LIB_DIR=lib

STM_LIB_SRC = Drivers/STM32F0xx_HAL_Driver
STM_USB_CORE = Middlewares/ST/STM32_USB_Device_Library/Core
STM_USB_CDC = Middlewares/ST/STM32_USB_Device_Library/Class/CDC
CMSIS_CORE = Drivers/CMSIS/Include
CMSIS_STM32 = Drivers/CMSIS/Device/ST/STM32F0xx

TARGET=stm32f0xx

# Put your source files here (or *.cflashing a hex file on a stm32f4discovery, etc)

USR_SRC := $(SRC_DIR)/*.c
CAN_SRC := $(CAN_DIR)/*.cpp
I2C_SRC := $(I2C_DIR)/*.cpp
STM_SRC := $(STM_LIB_SRC)/Src/$(TARGET)*.c
#STM_SRC += $(CMSIS_STM32)/Source/Templates/system_$(TARGET).c
STM_SRC += $(STM_USB_CORE)/Src/*.c
STM_SRC += $(STM_USB_CDC)/Src/*.c
STARTUP := startup_stm32f042x6.s

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=CanNode

# Normally you shouldn't need to change anything below this line!
#######################################################################################
TOOLPATH=/home/ryan/Downloads/gcc-arm-none-eabi-6-2017-q2-update/bin/
CC=$(TOOLPATH)arm-none-eabi-gcc
CXX=$(TOOLPATH)arm-none-eabi-g++
AR=$(TOOLPATH)arm-none-eabi-ar
OBJCOPY=$(TOOLPATH)arm-none-eabi-objcopy

ODIR=obj

C_DEFS := -DUSE_HAL_DRIVER -DSTM32F042x6

FLAGS += -Os -Wall -g $(C_DEFS)
FLAGS += --specs=nosys.specs -mthumb -mcpu=cortex-m0
FLAGS += -TSTM32F042C6Tx_FLASH.ld -fdata-sections -ffunction-sections -Wl,--gc-sections
CFLAGS = --std=gnu11  $(FLAGS)
CPPFLAGS = --std=gnu++11 $(FLAGS)

# Include files from STM libraries
INCLUDE += -I$(INC_DIR)
INCLUDE += -I$(CMSIS_CORE)
INCLUDE += -I$(CMSIS_STM32)/Include
INCLUDE += -I$(STM_LIB_SRC)/Inc
INCLUDE += -I$(STM_USB_CORE)/Inc
INCLUDE += -I$(STM_USB_CDC)/Inc

STM_SRC_EXP := $(wildcard $(STM_SRC))
STM_OBJ := $(STM_SRC_EXP:.c=.o)
STM_OBJ += $(STARTUP:.s=.o)

CAN_SRC_EXP := $(wildcard $(CAN_SRC))
CAN_OBJ := $(CAN_SRC_EXP:.cpp=.o)

I2C_SRC_EXP := $(wildcard $(I2C_SRC))
I2C_OBJ := $(I2C_SRC_EXP:.cpp=.o)

SRC_EXP := $(wildcard $(USR_SRC))
SRC_OBJ := $(SRC_EXP:.c=.o)

.PHONY: clean all size dfu

.c.o:
	$(CC) -c $(INCLUDE) $(CFLAGS)  $< -o $@

.cpp.o:
	$(CXX) -c $(INCLUDE) $(CPPFLAGS)  $< -o $@

.s.o:
	$(CC) -c $(INCLUDE) $(CFLAGS)  $< -o $@

all: main size

main: $(CAN_OBJ) $(I2C_OBJ) $(STM_OBJ) $(SRC_OBJ)  main.o
	$(CXX) $(CPPFLAGS) $(INCLUDE) $(SRC_OBJ) $(CAN_OBJ) $(I2C_OBJ) $(STM_OBJ) main.o -o $(PROJ_NAME).elf
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex

	
CanNode: $(CAN_OBJ) $(STM_OBJ)
	$(AR) rcs $(LIB_DIR)/libCanNode.a $(STM_OBJ) $(CAN_OBJ)

StmCore: $(STM_OBJ)
	$(AR) rcs $(LIB_DIR)/libStmCore.a $^

clean:
	rm -f *.o $(CAN_OBJ) $(I2C_OBJ) $(STM_OBJ) $(SRC_OBJ) $(PROJ_NAME)*.elf $(PROJ_NAME)*.bin

size: 
	arm-none-eabi-size $(PROJ_NAME)*.elf

tags: force_look
	ctags -R *

force_look:
	true

# Flash the STM32F4
flash: all
	dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000:leave -D $(PROJ_NAME).bin

stflash: main
	st-flash write $(PROJ_NAME).bin 0x08000000

docs:
	doxygen Doxyfile
