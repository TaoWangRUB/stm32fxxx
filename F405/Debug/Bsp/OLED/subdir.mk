################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/OLED/fonts.c \
../Bsp/OLED/ssd1306.c 

OBJS += \
./Bsp/OLED/fonts.o \
./Bsp/OLED/ssd1306.o 

C_DEPS += \
./Bsp/OLED/fonts.d \
./Bsp/OLED/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/OLED/%.o Bsp/OLED/%.su Bsp/OLED/%.cyclo: ../Bsp/OLED/%.c Bsp/OLED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Core -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Bsp -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-OLED

clean-Bsp-2f-OLED:
	-$(RM) ./Bsp/OLED/fonts.cyclo ./Bsp/OLED/fonts.d ./Bsp/OLED/fonts.o ./Bsp/OLED/fonts.su ./Bsp/OLED/ssd1306.cyclo ./Bsp/OLED/ssd1306.d ./Bsp/OLED/ssd1306.o ./Bsp/OLED/ssd1306.su

.PHONY: clean-Bsp-2f-OLED

