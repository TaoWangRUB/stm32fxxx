################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/IMU10DOF/Waveshare_10Dof-D.c 

OBJS += \
./Bsp/IMU10DOF/Waveshare_10Dof-D.o 

C_DEPS += \
./Bsp/IMU10DOF/Waveshare_10Dof-D.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/IMU10DOF/%.o Bsp/IMU10DOF/%.su Bsp/IMU10DOF/%.cyclo: ../Bsp/IMU10DOF/%.c Bsp/IMU10DOF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Core -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-IMU10DOF

clean-Bsp-2f-IMU10DOF:
	-$(RM) ./Bsp/IMU10DOF/Waveshare_10Dof-D.cyclo ./Bsp/IMU10DOF/Waveshare_10Dof-D.d ./Bsp/IMU10DOF/Waveshare_10Dof-D.o ./Bsp/IMU10DOF/Waveshare_10Dof-D.su

.PHONY: clean-Bsp-2f-IMU10DOF

