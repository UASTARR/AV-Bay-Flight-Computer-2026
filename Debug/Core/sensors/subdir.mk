################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/sensors/bme680.c \
../Core/sensors/bme68x.c \
../Core/sensors/icm40609d.c \
../Core/sensors/mmc5983ma.c \
../Core/sensors/ms5611.c 

OBJS += \
./Core/sensors/bme680.o \
./Core/sensors/bme68x.o \
./Core/sensors/icm40609d.o \
./Core/sensors/mmc5983ma.o \
./Core/sensors/ms5611.o 

C_DEPS += \
./Core/sensors/bme680.d \
./Core/sensors/bme68x.d \
./Core/sensors/icm40609d.d \
./Core/sensors/mmc5983ma.d \
./Core/sensors/ms5611.d 


# Each subdirectory must supply rules for building sources it contributes
Core/sensors/%.o Core/sensors/%.su Core/sensors/%.cyclo: ../Core/sensors/%.c Core/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F732xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-sensors

clean-Core-2f-sensors:
	-$(RM) ./Core/sensors/bme680.cyclo ./Core/sensors/bme680.d ./Core/sensors/bme680.o ./Core/sensors/bme680.su ./Core/sensors/bme68x.cyclo ./Core/sensors/bme68x.d ./Core/sensors/bme68x.o ./Core/sensors/bme68x.su ./Core/sensors/icm40609d.cyclo ./Core/sensors/icm40609d.d ./Core/sensors/icm40609d.o ./Core/sensors/icm40609d.su ./Core/sensors/mmc5983ma.cyclo ./Core/sensors/mmc5983ma.d ./Core/sensors/mmc5983ma.o ./Core/sensors/mmc5983ma.su ./Core/sensors/ms5611.cyclo ./Core/sensors/ms5611.d ./Core/sensors/ms5611.o ./Core/sensors/ms5611.su

.PHONY: clean-Core-2f-sensors

