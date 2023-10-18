################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
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
EXECUTABLES += \
LED\ Foot\ switch.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LED\ Foot\ switch.list \

OBJCOPY_BIN += \
LED\ Foot\ switch.bin \


# All Target
all: LED\ Foot\ switch.elf secondary-outputs

# Tool invocations
LED\ Foot\ switch.elf: $(OBJS) $(USER_OBJS) C:\Users\miyazaki\STM32CubeIDE\workspace_1.1.0\LED\ Foot\ switch\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "LED Foot switch.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\miyazaki\STM32CubeIDE\workspace_1.1.0\LED Foot switch\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LED Foot switch.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LED\ Foot\ switch.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LED Foot switch.list"
	@echo 'Finished building: $@'
	@echo ' '

LED\ Foot\ switch.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LED Foot switch.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
