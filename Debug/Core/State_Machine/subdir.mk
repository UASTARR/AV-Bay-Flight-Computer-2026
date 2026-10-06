################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/State_Machine/state.c 

OBJS += \
./Core/State_Machine/state.o 

C_DEPS += \
./Core/State_Machine/state.d 


# Each subdirectory must supply rules for building sources it contributes
Core/State_Machine/%.o Core/State_Machine/%.su Core/State_Machine/%.cyclo: ../Core/State_Machine/%.c Core/State_Machine/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F732xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-State_Machine

clean-Core-2f-State_Machine:
	-$(RM) ./Core/State_Machine/state.cyclo ./Core/State_Machine/state.d ./Core/State_Machine/state.o ./Core/State_Machine/state.su

.PHONY: clean-Core-2f-State_Machine

