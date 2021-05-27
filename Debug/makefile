################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
21573751Arcade.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
21573751Arcade.list \

OBJCOPY_BIN += \
21573751Arcade.bin \


# All Target
all: 21573751Arcade.elf secondary-outputs

# Tool invocations
21573751Arcade.elf: $(OBJS) $(USER_OBJS) C:\Users\User\STM32CubeIDE\workspace_1.3.0\21573751Arcade\STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "21573751Arcade.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\User\STM32CubeIDE\workspace_1.3.0\21573751Arcade\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="21573751Arcade.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

21573751Arcade.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "21573751Arcade.list"
	@echo 'Finished building: $@'
	@echo ' '

21573751Arcade.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "21573751Arcade.bin"
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
