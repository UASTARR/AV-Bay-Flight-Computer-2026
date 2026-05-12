################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/software_i2c/dwt_stm32_delay.c \
../Core/software_i2c/stm32_sw_i2c.c 

OBJS += \
./Core/software_i2c/dwt_stm32_delay.o \
./Core/software_i2c/stm32_sw_i2c.o 

C_DEPS += \
./Core/software_i2c/dwt_stm32_delay.d \
./Core/software_i2c/stm32_sw_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Core/software_i2c/%.o Core/software_i2c/%.su Core/software_i2c/%.cyclo: ../Core/software_i2c/%.c Core/software_i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F732xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/alexk/Desktop/FlightComputer/Core/sensors" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-software_i2c

clean-Core-2f-software_i2c:
	-$(RM) ./Core/software_i2c/dwt_stm32_delay.cyclo ./Core/software_i2c/dwt_stm32_delay.d ./Core/software_i2c/dwt_stm32_delay.o ./Core/software_i2c/dwt_stm32_delay.su ./Core/software_i2c/stm32_sw_i2c.cyclo ./Core/software_i2c/stm32_sw_i2c.d ./Core/software_i2c/stm32_sw_i2c.o ./Core/software_i2c/stm32_sw_i2c.su

.PHONY: clean-Core-2f-software_i2c

