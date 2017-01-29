PROJECT = f051c8tx
 
DIR=~/gcc-arm/bin
CC=$(DIR)/arm-none-eabi-gcc
NM=$(DIR)/arm-none-eabi-nm
CP=$(DIR)/arm-none-eabi-objcopy
OD=$(DIR)/arm-none-eabi-objdump
SZ=$(DIR)/arm-none-eabi-size

# SEMIHOST_LIB = --specs=rdimon.specs -lrdimon
NANOLIB = --specs=nano.specs

OPENOCD=~/openocd
TARGETCNF=stm32f0x.cfg
INTERFACECNF=cmsis-dap.cfg

DEFS = -DUSE_HAL_DRIVER -DSTM32F051x8

MCU = cortex-m0
MCFLAGS  = -MD -mcpu=$(MCU) -march=armv6-m -mtune=$(MCU)
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
MCFLAGS += -nostartfiles
MCFLAGS += -Wl,--gc-sections
# MCFLAGS += -u _printf_float

STM32_INCLUDES = \
	-IInc \
	-IDrivers/CMSIS/Device/ST/STM32F0xx/Include \
	-IDrivers/CMSIS/Include \
	-IDrivers/STM32F0xx_HAL_Driver/Inc/Legacy \
	-IDrivers/STM32F0xx_HAL_Driver/Inc

#OPTIMIZE = -Os
OPTIMIZE = -gdwarf-2 -O0 -flto-partition=none -fipa-sra

CFLAGS  = $(MCFLAGS) $(OPTIMIZE) $(NANOLIB) $(SEMIHOST_LIB) $(DEFS) $(STM32_INCLUDES)
CFLAGS += -Wl,-T,STM32F051C8Tx_FLASH.ld
AFLAGS  = $(MCFLAGS)

CSRC = \
	$(wildcard Src/*.c) \
	$(wildcard Drivers/STM32F0xx_HAL_Driver/Src/*.c)

ASRC = Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f051x8.s

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
	$(OPENOCD)/src/openocd -s $(OPENOCD)/tcl -f interface/$(INTERFACECNF) -f target/$(TARGETCNF) -c "program $^ verify reset exit"