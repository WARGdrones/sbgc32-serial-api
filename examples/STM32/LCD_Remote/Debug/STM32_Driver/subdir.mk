################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: ARM_TC
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/basecam/SBGC32_Library/sbgc32-serial-api/drivers/STM32_Driver/driver_STM32.c 

OBJS += \
./STM32_Driver/driver_STM32.o 

C_DEPS += \
./STM32_Driver/driver_STM32.d 


# Each subdirectory must supply rules for building sources it contributes
STM32_Driver/driver_STM32.o: D:/basecam/SBGC32_Library/sbgc32-serial-api/drivers/STM32_Driver/driver_STM32.c STM32_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F767xx -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../../../../sources -I../../../../drivers/STM32_Driver -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32_Driver

clean-STM32_Driver:
	-$(RM) ./STM32_Driver/driver_STM32.d ./STM32_Driver/driver_STM32.o ./STM32_Driver/driver_STM32.su

.PHONY: clean-STM32_Driver

