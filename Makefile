BINARY		= motor-driver
PREFIX		?= arm-none-eabi
CC		= $(PREFIX)-gcc
LD		= $(PREFIX)-gcc
AS		= $(PREFIX)-as
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
ARCH_FLAGS	=  -mcpu=cortex-m0 -mthumb -msoft-float
INCLUDES	=  -Iinc -Ilib/STM32CubeG0/Drivers/STM32G0xx_HAL_Driver/Inc/ \
		   -Ilib/STM32CubeG0/Drivers/CMSIS/Device/ST/STM32G0xx/Include/ \
		   -Ilib/STM32CubeG0/Drivers/CMSIS/Include/
CFLAGS		+= -Os -g -Wall -Wextra $(INCLUDES) $(ARCH_FLAGS) \
		   -fno-common -MD -DSTM32G071xx -DUSE_FULL_LL_DRIVER -fno-builtin \
		   -std=gnu99 --specs=nano.specs
LDSCRIPT	?= STM32G071KBTX_FLASH.ld
LDFLAGS		+= -Wl,--start-group -lc -lgcc -Wl,--end-group \
		   -Llib/libopencm3/lib \
		   -T$(LDSCRIPT) -Wl,--gc-sections \
		   $(ARCH_FLAGS)
BUILD_DIR	=  build/
OBJ_FILES	+= src/main.o src/startup_stm32g071rbtx.o \
		   src/stm32g0xx_it.o src/syscalls.o src/sysmem.o \
		   src/system_stm32g0xx.o
DRIVER_OBJ_FILES = stm32g0xx_ll_adc.o stm32g0xx_ll_comp.o stm32g0xx_ll_crc.o \
		   stm32g0xx_ll_crs.o stm32g0xx_ll_dac.o stm32g0xx_ll_dma.o \
		   stm32g0xx_ll_exti.o stm32g0xx_ll_gpio.o stm32g0xx_ll_i2c.o \
		   stm32g0xx_ll_lptim.o stm32g0xx_ll_lpuart.o stm32g0xx_ll_pwr.o \
		   stm32g0xx_ll_rcc.o stm32g0xx_ll_rng.o stm32g0xx_ll_rtc.o \
		   stm32g0xx_ll_spi.o stm32g0xx_ll_tim.o stm32g0xx_ll_ucpd.o \
		   stm32g0xx_ll_usart.o stm32g0xx_ll_utils.o
OBJS		=  $(OBJ_FILES:%.o=$(BUILD_DIR)%.o) $(addprefix $(BUILD_DIR)lib/STM32CubeG0/Drivers/STM32G0xx_HAL_Driver/Src/,$(DRIVER_OBJ_FILES))

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS += -Wl,--print-gc-sections
endif

.SUFFIXES: .elf .bin .hex .srec .list .images
.SECONDARY:

all: images

objs:
	$(Q)echo $(OBJS)

images: $(BINARY).bin $(BINARY).hex

asm: $(BINARY).asm

%.bin: $(BUILD_DIR)%.elf
	$(Q)$(OBJCOPY) -Obinary $(BUILD_DIR)$(*).elf $(*).bin

%.hex: $(BUILD_DIR)%.elf
	$(Q)$(OBJCOPY) -Oihex $(BUILD_DIR)$(*).elf $(*).hex

%.asm: $(BUILD_DIR)%.elf
	$(Q)$(OBJDUMP) -d $(BUILD_DIR)$(*).elf > $(*).asm

%.list: $(BUILD_DIR)%.elf
	$(Q)$(OBJDUMP) -S $(BUILD_DIR)$(*).elf > $(*).list

%.elf: $(OBJS) $(LDSCRIPT) Makefile
	$(Q)$(LD) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR):
	$(Q)mkdir -p $@

$(BUILD_DIR)%/.:
	$(Q)mkdir -p $@

flash: $(BINARY).hex
	$(Q)openocd -f interface/ftdi/olimex-arm-usb-ocd.cfg \
		-f interface/ftdi/olimex-arm-jtag-swd.cfg \
		-c "source [find target/stm32g0x.cfg]" \
		-c "init" -c "reset init" \
		-c "flash write_image erase $(BINARY).hex" \
		-c "reset" -c "shutdown" $(NULL)

clean:
	$(Q)rm -f *.o
	$(Q)rm -f *.d
	$(Q)rm -f *.elf
	$(Q)rm -f *.bin
	$(Q)rm -f *.hex
	$(Q)rm -f *.srec
	$(Q)rm -f *.list
	$(Q)rm -rf $(BUILD_DIR)

.PHONY: images clean

.SECONDEXPANSION:

$(BUILD_DIR)%.o: %.s Makefile | $(BUILD_DIR) $$(@D)/.
	$(Q)$(AS) -o $@ $<

$(BUILD_DIR)%.o: %.c Makefile | $(BUILD_DIR) $$(@D)/.
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

-include $(OBJS:.o=.d)
