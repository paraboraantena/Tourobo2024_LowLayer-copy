################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.c \
../CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.c 

OBJS += \
./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.o \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.o 

C_DEPS += \
./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.d \
./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/MatrixFunctions/%.o CMSIS-DSP/Source/MatrixFunctions/%.su CMSIS-DSP/Source/MatrixFunctions/%.cyclo: ../CMSIS-DSP/Source/MatrixFunctions/%.c CMSIS-DSP/Source/MatrixFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -I../CMSIS-DSP/ComputeLibrary/Include -I../Core/Inc -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-MatrixFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-MatrixFunctions:
	-$(RM) ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_householder_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_add_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.d
	-$(RM) ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_q7.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_qr_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_scale_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.cyclo
	-$(RM) ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_sub_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f64.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_q7.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.su ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.cyclo ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.d ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.o ./CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-MatrixFunctions

