PROJECT = stm32l151
 
DIR=~/gcc-arm/bin
CC=$(DIR)/arm-none-eabi-gcc
NM=$(DIR)/arm-none-eabi-nm
CP=$(DIR)/arm-none-eabi-objcopy
OD=$(DIR)/arm-none-eabi-objdump
SZ=$(DIR)/arm-none-eabi-size

# SEMIHOST_LIB = --specs=rdimon.specs -lrdimon
NANOLIB = --specs=nosys.specs --specs=nano.specs

OPENOCD=~/openocd
TARGETCNF=stm32l1.cfg
#INTERFACECNF=stlink-v2-1.cfg
INTERFACECNF=cmsis-dap.cfg

DEFS = -DUSE_HAL_DRIVER -DSTM32L151xB -DGCC_ARMCM3

MCU = cortex-m3
MCFLAGS  = -MD -mcpu=$(MCU) -march=armv7-m -mtune=$(MCU)
MCFLAGS += -std=gnu99
MCFLAGS += -mthumb -mlittle-endian -mno-unaligned-access -mno-sched-prolog
# MCFLAGS += -mfloat-abi=hard  -mfpu=fpv4-sp-d16
MCFLAGS += -fno-strict-aliasing -fsigned-char
MCFLAGS += -ffunction-sections -fdata-sections
MCFLAGS += -fno-schedule-insns2
MCFLAGS += -fno-common -fno-hosted
MCFLAGS += -mthumb-interwork
MCFLAGS += -fmessage-length=0
MCFLAGS += -ffreestanding
MCFLAGS += -fno-move-loop-invariants
MCFLAGS += -fsingle-precision-constant
# MCFLAGS += -nostartfiles
MCFLAGS += -Wl,--gc-sections
# MCFLAGS += -u _printf_float

STM32_INCLUDES = \
	-IInc \
	-IDrivers/CMSIS/Device/ST/STM32L1xx/Include \
	-IDrivers/CMSIS/Include \
	-IDrivers/STM32L1xx_HAL_Driver/Inc/Legacy \
	-IDrivers/STM32L1xx_HAL_Driver/Inc \
	-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3

# OPTIMIZE = -gdwarf-2 -O0 -flto-partition=none -fipa-sra
OPTIMIZE = -Os -finline -finline-functions-called-once

CFLAGS  = $(MCFLAGS) $(OPTIMIZE) $(NANOLIB) $(SEMIHOST_LIB) $(DEFS) $(STM32_INCLUDES)
CFLAGS += -Wl,-cref -T STM32L151C8Tx_FLASH.ld
AFLAGS  = $(MCFLAGS)

CSRC = \
	$(wildcard Src/*.c) \
	$(wildcard Drivers/STM32L1xx_HAL_Driver/Src/*.c) \
	$(wildcard Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/*.c) \
	$(wildcard Middlewares/Third_Party/FreeRTOS/Source/*.c) \
	$(wildcard Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/*.c) \
	$(wildcard Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/*.c) \
	$(wildcard Middlewares/Third_Party/FreeRTOS/Source/portable/*.c)

ASRC = startup/startup_stm32l151xb.s

COBJ = $(CSRC:.c=.o)
AOBJ = $(ASRC:.s=.o)

all: build size

build: elf hex lst sym

elf: $(PROJECT).elf
hex: $(PROJECT).hex
lst: $(PROJECT).lst
sym: $(PROJECT).sym

%.hex: %.elf
	@echo
	$(CP) -O ihex $< $@

%.lst: %.elf
	@echo
	$(OD) -h -S -C $< > $@

%.sym: %.elf
	@echo
	$(NM) -n $< > $@

size:
	@echo
	$(SZ) $(PROJECT).hex
	$(SZ) $(PROJECT).elf
	$(SZ) -A -x $(PROJECT).elf

%.elf: $(AOBJ) $(COBJ)
	@echo
	$(CC) $(CFLAGS) $^ -o $@

$(COBJ) : %.o : %.c
	@echo
	$(CC) -c $(CFLAGS) $< -o $@

$(AOBJ) : %.o : %.s
	@echo
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	@echo
	find . -name '*.o' -print -or -name '*.d' -print | xargs rm -f
	rm -f $(PROJECT).elf $(PROJECT).hex $(PROJECT).lst $(PROJECT).sym

flash: $(PROJECT).elf
#	$(OPENOCD)/src/openocd -s $(OPENOCD)/tcl -f board/st_nucleo_l1.cfg -c "program $^ verify reset exit"

	$(OPENOCD)/src/openocd -s $(OPENOCD)/tcl -f interface/$(INTERFACECNF) -f target/$(TARGETCNF) -c "program $^ verify reset exit"
