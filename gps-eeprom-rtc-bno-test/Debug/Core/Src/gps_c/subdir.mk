################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/gps_c/NMEA.cpp \
../Core/Src/gps_c/uartRingBuffer.cpp 

OBJS += \
./Core/Src/gps_c/NMEA.o \
./Core/Src/gps_c/uartRingBuffer.o 

CPP_DEPS += \
./Core/Src/gps_c/NMEA.d \
./Core/Src/gps_c/uartRingBuffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/gps_c/%.o Core/Src/gps_c/%.su Core/Src/gps_c/%.cyclo: ../Core/Src/gps_c/%.cpp Core/Src/gps_c/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-gps_c

clean-Core-2f-Src-2f-gps_c:
	-$(RM) ./Core/Src/gps_c/NMEA.cyclo ./Core/Src/gps_c/NMEA.d ./Core/Src/gps_c/NMEA.o ./Core/Src/gps_c/NMEA.su ./Core/Src/gps_c/uartRingBuffer.cyclo ./Core/Src/gps_c/uartRingBuffer.d ./Core/Src/gps_c/uartRingBuffer.o ./Core/Src/gps_c/uartRingBuffer.su

.PHONY: clean-Core-2f-Src-2f-gps_c

