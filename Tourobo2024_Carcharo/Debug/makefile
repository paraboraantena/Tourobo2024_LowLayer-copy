################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/LwIP/system/OS/subdir.mk
-include Middlewares/Third_Party/LwIP/src/netif/ppp/subdir.mk
-include Middlewares/Third_Party/LwIP/src/netif/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/ipv6/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/ipv4/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/subdir.mk
-include Middlewares/Third_Party/LwIP/src/apps/mqtt/subdir.mk
-include Middlewares/Third_Party/LwIP/src/api/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include LWIP/Target/subdir.mk
-include LWIP/App/subdir.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/lan8742/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include CMSIS-DSP/Source/WindowFunctions/subdir.mk
-include CMSIS-DSP/Source/TransformFunctions/subdir.mk
-include CMSIS-DSP/Source/SupportFunctions/subdir.mk
-include CMSIS-DSP/Source/StatisticsFunctions/subdir.mk
-include CMSIS-DSP/Source/SVMFunctions/subdir.mk
-include CMSIS-DSP/Source/QuaternionMathFunctions/subdir.mk
-include CMSIS-DSP/Source/MatrixFunctions/subdir.mk
-include CMSIS-DSP/Source/InterpolationFunctions/subdir.mk
-include CMSIS-DSP/Source/FilteringFunctions/subdir.mk
-include CMSIS-DSP/Source/FastMathFunctions/subdir.mk
-include CMSIS-DSP/Source/DistanceFunctions/subdir.mk
-include CMSIS-DSP/Source/ControllerFunctions/subdir.mk
-include CMSIS-DSP/Source/ComplexMathFunctions/subdir.mk
-include CMSIS-DSP/Source/CommonTables/subdir.mk
-include CMSIS-DSP/Source/BayesFunctions/subdir.mk
-include CMSIS-DSP/Source/BasicMathFunctions/subdir.mk
-include CMSIS-DSP/ComputeLibrary/Source/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Tourobo2024_Carcharo
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Tourobo2024_Carcharo.elf \

MAP_FILES += \
Tourobo2024_Carcharo.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Tourobo2024_Carcharo.list \


# All Target
all: main-build

# Main-build Target
main-build: Tourobo2024_Carcharo.elf secondary-outputs

# Tool invocations
Tourobo2024_Carcharo.elf Tourobo2024_Carcharo.map: $(OBJS) $(USER_OBJS) C:\Users\harut\OneDrive\ -\ OUMail\ (Osaka\ University)\Robohan\Tourobo2024_LowLayer\Tourobo2024_Carcharo\STM32F767ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Tourobo2024_Carcharo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\harut\OneDrive - OUMail (Osaka University)\Robohan\Tourobo2024_LowLayer\Tourobo2024_Carcharo\STM32F767ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Tourobo2024_Carcharo.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Tourobo2024_Carcharo.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Tourobo2024_Carcharo.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Tourobo2024_Carcharo.elf Tourobo2024_Carcharo.list Tourobo2024_Carcharo.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
