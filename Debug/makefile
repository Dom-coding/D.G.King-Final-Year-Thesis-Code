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
Final_Year_Thesis.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Final_Year_Thesis.list \

OBJCOPY_BIN += \
Final_Year_Thesis.bin \


# All Target
all: Final_Year_Thesis.elf secondary-outputs

# Tool invocations
Final_Year_Thesis.elf: $(OBJS) $(USER_OBJS) C:\Users\Dominic\ King\Documents\4th\ Year\ Engineering\Skripsie\STM\ code\STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "Final_Year_Thesis.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Dominic King\Documents\4th Year Engineering\Skripsie\STM code\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Final_Year_Thesis.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Final_Year_Thesis.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Final_Year_Thesis.list"
	@echo 'Finished building: $@'
	@echo ' '

Final_Year_Thesis.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Final_Year_Thesis.bin"
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
