################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/RTClib_c/RTClib.cpp 

OBJS += \
./Core/Src/RTClib_c/RTClib.o 

CPP_DEPS += \
./Core/Src/RTClib_c/RTClib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/RTClib_c/%.o Core/Src/RTClib_c/%.su Core/Src/RTClib_c/%.cyclo: ../Core/Src/RTClib_c/%.cpp Core/Src/RTClib_c/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fexceptions -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-RTClib_c

clean-Core-2f-Src-2f-RTClib_c:
	-$(RM) ./Core/Src/RTClib_c/RTClib.cyclo ./Core/Src/RTClib_c/RTClib.d ./Core/Src/RTClib_c/RTClib.o ./Core/Src/RTClib_c/RTClib.su

.PHONY: clean-Core-2f-Src-2f-RTClib_c

