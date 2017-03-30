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
boot_api_app.lss \

FLASH_IMAGE += \
boot_api_app.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: boot_api_app.elf secondary-outputs

# Tool invocations
boot_api_app.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,boot_api_app.map -mmcu=atmega328p -o "boot_api_app.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

boot_api_app.lss: boot_api_app.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S boot_api_app.elf  >"boot_api_app.lss"
	@echo 'Finished building: $@'
	@echo ' '

boot_api_app.hex: boot_api_app.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex boot_api_app.elf  "boot_api_app.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: boot_api_app.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p boot_api_app.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) boot_api_app.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
