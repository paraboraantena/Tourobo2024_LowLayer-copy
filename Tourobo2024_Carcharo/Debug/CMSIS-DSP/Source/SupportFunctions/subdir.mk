################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.c 

OBJS += \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.o 

C_DEPS += \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/SupportFunctions/%.o CMSIS-DSP/Source/SupportFunctions/%.su CMSIS-DSP/Source/SupportFunctions/%.cyclo: ../CMSIS-DSP/Source/SupportFunctions/%.c CMSIS-DSP/Source/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -I../CMSIS-DSP/ComputeLibrary/Include -I../Core/Inc -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-SupportFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-SupportFunctions:
	-$(RM) ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.d
	-$(RM) ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_sum_f32.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-SupportFunctions

