################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/EKF/prediction.c 

OBJS += \
./Core/EKF/prediction.o 

C_DEPS += \
./Core/EKF/prediction.d 


# Each subdirectory must supply rules for building sources it contributes
Core/EKF/%.o Core/EKF/%.su Core/EKF/%.cyclo: ../Core/EKF/%.c Core/EKF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F732xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-EKF

clean-Core-2f-EKF:
	-$(RM) ./Core/EKF/prediction.cyclo ./Core/EKF/prediction.d ./Core/EKF/prediction.o ./Core/EKF/prediction.su

.PHONY: clean-Core-2f-EKF

