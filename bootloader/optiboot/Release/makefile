################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
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
optiboot.lss \

FLASH_IMAGE += \
optiboot.hex \

EEPROM_IMAGE += \
optiboot.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: optiboot.elf secondary-outputs

# Tool invocations
optiboot.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,optiboot.map -Wl,--section-start=.text=0x3f00 -Wl,--section-start=.version=0x3fff -mmcu=atmega328p -o "optiboot.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

optiboot.lss: optiboot.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S optiboot.elf  >"optiboot.lss"
	@echo 'Finished building: $@'
	@echo ' '

optiboot.hex: optiboot.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex optiboot.elf  "optiboot.hex"
	@echo 'Finished building: $@'
	@echo ' '

optiboot.eep: optiboot.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex optiboot.elf  "optiboot.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: optiboot.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p optiboot.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) optiboot.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
