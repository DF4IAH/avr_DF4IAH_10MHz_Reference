################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/usbdrv_fw/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
avr_DF4IAH_10MHz_Reference.lss \

FLASH_IMAGE += \
avr_DF4IAH_10MHz_Reference.hex \

EEPROM_IMAGE += \
avr_DF4IAH_10MHz_Reference.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: avr_DF4IAH_10MHz_Reference.elf secondary-outputs

dependents:
	-cd /home/espero/git/avr_DF4IAH_Bootloader/Debug && $(MAKE) clean all

# Tool invocations
avr_DF4IAH_10MHz_Reference.elf: $(OBJS) $(USER_OBJS) /home/espero/git/avr_DF4IAH_Bootloader/Debug/avr_DF4IAH_Bootloader.elf
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,avr_DF4IAH_10MHz_Reference.map -Wl,--section-start=.text=0x0000 -mmcu=atmega328p -o "avr_DF4IAH_10MHz_Reference.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

avr_DF4IAH_10MHz_Reference.lss: avr_DF4IAH_10MHz_Reference.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S avr_DF4IAH_10MHz_Reference.elf  >"avr_DF4IAH_10MHz_Reference.lss"
	@echo 'Finished building: $@'
	@echo ' '

avr_DF4IAH_10MHz_Reference.hex: avr_DF4IAH_10MHz_Reference.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex avr_DF4IAH_10MHz_Reference.elf  "avr_DF4IAH_10MHz_Reference.hex"
	@echo 'Finished building: $@'
	@echo ' '

avr_DF4IAH_10MHz_Reference.eep: avr_DF4IAH_10MHz_Reference.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex avr_DF4IAH_10MHz_Reference.elf  "avr_DF4IAH_10MHz_Reference.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: avr_DF4IAH_10MHz_Reference.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=berkeley -t avr_DF4IAH_10MHz_Reference.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) avr_DF4IAH_10MHz_Reference.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
/home/espero/git/avr_DF4IAH_Bootloader/Debug/avr_DF4IAH_Bootloader.elf:

-include ../makefile.targets
