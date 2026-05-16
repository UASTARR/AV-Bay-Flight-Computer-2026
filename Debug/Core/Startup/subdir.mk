################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f732retx.s 

OBJS += \
./Core/Startup/startup_stm32f732retx.o 

S_DEPS += \
./Core/Startup/startup_stm32f732retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g -DDEBUG -DARM_MATH_CM7 -c -I"C:/Users/alexk/Desktop/FlightComputer/Drivers/CMSIS/Include" -I"C:/Users/alexk/Desktop/FlightComputer/Core/Inc" -I"C:/Users/alexk/Desktop/FlightComputer/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f732retx.d ./Core/Startup/startup_stm32f732retx.o

.PHONY: clean-Core-2f-Startup

