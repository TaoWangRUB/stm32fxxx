################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/MPU6050/mpu6050.c 

OBJS += \
./Bsp/MPU6050/mpu6050.o 

C_DEPS += \
./Bsp/MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/MPU6050/%.o Bsp/MPU6050/%.su Bsp/MPU6050/%.cyclo: ../Bsp/MPU6050/%.c Bsp/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Core -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-MPU6050

clean-Bsp-2f-MPU6050:
	-$(RM) ./Bsp/MPU6050/mpu6050.cyclo ./Bsp/MPU6050/mpu6050.d ./Bsp/MPU6050/mpu6050.o ./Bsp/MPU6050/mpu6050.su

.PHONY: clean-Bsp-2f-MPU6050

