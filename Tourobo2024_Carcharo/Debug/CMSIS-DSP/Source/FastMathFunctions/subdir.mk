################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.c \
../CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.c 

OBJS += \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.o \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.o 

C_DEPS += \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.d \
./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/FastMathFunctions/%.o CMSIS-DSP/Source/FastMathFunctions/%.su CMSIS-DSP/Source/FastMathFunctions/%.cyclo: ../CMSIS-DSP/Source/FastMathFunctions/%.c CMSIS-DSP/Source/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -I../CMSIS-DSP/ComputeLibrary/Include -I../Core/Inc -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-FastMathFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-FastMathFunctions:
	-$(RM) ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.d ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.o ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f16.su ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.d ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.o ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.su ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.d ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.o ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q15.su ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.d ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.o ./CMSIS-DSP/Source/FastMathFunctions/arm_atan2_q31.su ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.d ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.o ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.su ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.d ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.o ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q15.su ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.d ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.o ./CMSIS-DSP/Source/FastMathFunctions/arm_cos_q31.su ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.d ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.o ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q15.su ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.d ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.o ./CMSIS-DSP/Source/FastMathFunctions/arm_divide_q31.su ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.d ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.o ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.su ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.d ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.o ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q15.su ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.d ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.o ./CMSIS-DSP/Source/FastMathFunctions/arm_sin_q31.su ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.d ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.o ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q15.su ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.d ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.o ./CMSIS-DSP/Source/FastMathFunctions/arm_sqrt_q31.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f16.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f32.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vexp_f64.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vinverse_f16.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f16.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f32.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_f64.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q15.su ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.cyclo ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.d ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.o ./CMSIS-DSP/Source/FastMathFunctions/arm_vlog_q31.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-FastMathFunctions

