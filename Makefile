# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = $(SHELL) -c "mkdir -p \"$(1)\""
    RM = $(SHELL) -c "rm -rf \"$(1)\""
endif

ifeq (,$(filter .build,$(notdir $(CURDIR))))
.SUFFIXES:
OBJDIR := .build
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = $(MAKE) --no-print-directory -C $(OBJDIR) -f $(mkfile_path) \
		SRCDIR=$(CURDIR) $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

VPATH = .. 

GCC_BIN =
PROJECT = frdm_MPU9250
OBJECTS = ./main.o 
SYS_OBJECTS = \
	./mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/board.o \
	./mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/cmsis_nvic.o \
	./mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/mbed_overrides.o \
	./mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/retarget.o \
	./mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/startup_MK64F12.o \
	./mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/system_MK64F12.o

INCLUDE_PATHS = \
	-I../. \
	-I.././include  \
	-I.././mbed \
	-I.././mbed/TARGET_K64F \
	-I.././mbed/TARGET_K64F/TARGET_Freescale \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/common \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/common/phyksz8081 \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/drivers \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/drivers/clock \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/drivers/enet \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/drivers/interrupt \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/drivers/pit \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/drivers/pit/common \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/adc \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/can \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/dac \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/dmamux \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/dspi \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/edma \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/enet \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/flextimer \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/gpio \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/i2c \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/llwu \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/lptmr \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/lpuart \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/mcg \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/mpu \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/osc \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/pdb \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/pit \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/pmc \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/port \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/rcm \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/rtc \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/sai \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/sdhc \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/sim \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/smc \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/uart \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/hal/wdog \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_KPSDK_CODE/utilities \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F/MK64F12 \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F/TARGET_FRDM \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F/device \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F/device/MK64F12 \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F/device/device \
	-I.././mbed/TARGET_K64F/TARGET_Freescale/TARGET_KPSDK_MCUS/TARGET_MCU_K64F/device/device/MK64F12 \
	-I.././mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM

LIBRARY_PATHS = -L.././mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM

LINK_LIB = -lmbed

LINKER_SCRIPT = .././mbed/TARGET_K64F/TOOLCHAIN_GCC_ARM/K64FN1M0xxx12.ld

############################################################################### 
AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE    = $(GCC_BIN)arm-none-eabi-size 

ifeq ($(HARDFP),1)
	FLOAT_ABI = hard
else
	FLOAT_ABI = softfp
endif


CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=$(FLOAT_ABI)

WARNING_FLAGS = -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers

COMPILE_FORCE_FLAGS = -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer

HW_CONFIG_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp

CC_FLAGS = -c  $(WARNING_FLAGS) $(COMPILE_FORCE_FLAGS) $(HW_CONFIG_FLAGS) -Os -std=c99 -include mbed_config.h -MMD -MP

CPPC_FLAGS = -c $(WARNING_FLAGS) $(COMPILE_FORCE_FLAGS) $(HW_CONFIG_FLAGS) -Os -std=c++11 -fno-rtti -Wvla -include mbed_config.h -MMD -MP

ASM_FLAGS = -x assembler-with-cpp -c $(WARNING_FLAGS) $(COMPILE_FORCE_FLAGS) $(HW_CONFIG_FLAGS) -Os

CC_SYMBOLS = \
	-D__MBED__=1 \
	-DDEVICE_I2CSLAVE=1 \
	-D__FPU_PRESENT=1 \
	-DTARGET_Freescale \
	-DDEVICE_PORTINOUT=1 \
	-DTARGET_RTOS_M4_M7 \
	-DDEVICE_RTC=1 \
	-DTOOLCHAIN_object \
	-D__CMSIS_RTOS -DFSL_RTOS_MBED \
	-DDEVICE_STORAGE=1 \
	-DMBEDTLS_ENTROPY_HARDWARE_ALT \
	-DTARGET_KPSDK_MCUS -DTOOLCHAIN_GCC \
	-DTARGET_CORTEX_M -DTARGET_KSDK2_MCUS \
	-DTARGET_LIKE_CORTEX_M4 -DDEVICE_ANALOGOUT=1 \
	-DTARGET_M4 -DTARGET_UVISOR_UNSUPPORTED -DTARGET_K64F \
	-DDEVICE_PWMOUT=1 -DDEVICE_INTERRUPTIN=1 -DDEVICE_I2C=1 \
	-DDEVICE_PORTOUT=1 -D__CORTEX_M4 -DDEVICE_STDIO_MESSAGES=1 \
	-DCPU_MK64FN1M0VMD12 -DFEATURE_IPV4=1 -DMBED_BUILD_TIMESTAMP=$(EPOCHREALTIME) \
	-DTARGET_FF_ARDUINO -DTARGET_KPSDK_CODE -DTARGET_RELEASE -DDEVICE_SERIAL_FC=1 \
	-DFEATURE_STORAGE=1 -DTARGET_LIKE_MBED -D__MBED_CMSIS_RTOS_CM -DDEVICE_SLEEP=1 \
	-DTOOLCHAIN_GCC_ARM -DTARGET_FRDM -DDEVICE_SPI=1 -DDEVICE_ERROR_RED=1 \
	-DDEVICE_SPISLAVE=1 -DDEVICE_ANALOGIN=1 -DDEVICE_SERIAL=1 -DDEVICE_PORTIN=1 \
	-DTARGET_MCU_K64F -DARM_MATH_CM4 

LD_FLAGS =-Wl,--gc-sections -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp 

LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys


ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -O0
else
  CC_FLAGS += -DNDEBUG -Os
endif


.PHONY: all lst size

all: $(PROJECT).bin $(PROJECT).hex size


.asm.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC) $(CPU) -c $(ASM_FLAGS) -o $@ $<
.s.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC) $(CPU) -c $(ASM_FLAGS) -o $@ $<
.S.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC) $(CPU) -c $(ASM_FLAGS) -o $@ $<

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CPP) $(CPPC_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(LINKER_SCRIPT)
	$(LD) $(LD_FLAGS) -T$(filter %.ld, $^) $(LIBRARY_PATHS) -o $@ $(filter %.o, $^) -Wl,--start-group $(LINK_LIB) $(LD_SYS_LIBS) -Wl,--end-group


$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size: $(PROJECT).elf
	$(SIZE) $(PROJECT).elf

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)


endif
