################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/attitude_estimation/attitude_estimation.c 

OBJS += \
./Core/attitude_estimation/attitude_estimation.o 

C_DEPS += \
./Core/attitude_estimation/attitude_estimation.d 


# Each subdirectory must supply rules for building sources it contributes
Core/attitude_estimation/%.o Core/attitude_estimation/%.su Core/attitude_estimation/%.cyclo: ../Core/attitude_estimation/%.c Core/attitude_estimation/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F732xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-attitude_estimation

clean-Core-2f-attitude_estimation:
	-$(RM) ./Core/attitude_estimation/attitude_estimation.cyclo ./Core/attitude_estimation/attitude_estimation.d ./Core/attitude_estimation/attitude_estimation.o ./Core/attitude_estimation/attitude_estimation.su

.PHONY: clean-Core-2f-attitude_estimation

