################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
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
eDesignMultimeter2022.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
eDesignMultimeter2022.list \

OBJCOPY_BIN += \
eDesignMultimeter2022.bin \


# All Target
all: eDesignMultimeter2022.elf secondary-outputs

# Tool invocations
eDesignMultimeter2022.elf: $(OBJS) $(USER_OBJS) C:\Repositories\eDesign\eDesignMultimeter2022\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "eDesignMultimeter2022.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Repositories\eDesign\eDesignMultimeter2022\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="eDesignMultimeter2022.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

eDesignMultimeter2022.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "eDesignMultimeter2022.list"
	@echo 'Finished building: $@'
	@echo ' '

eDesignMultimeter2022.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "eDesignMultimeter2022.bin"
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
