################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/libnova_c/aberration.c \
../Core/Src/libnova_c/airmass.c \
../Core/Src/libnova_c/angular_separation.c \
../Core/Src/libnova_c/apparent_position.c \
../Core/Src/libnova_c/asteroid.c \
../Core/Src/libnova_c/comet.c \
../Core/Src/libnova_c/dynamical_time.c \
../Core/Src/libnova_c/earth.c \
../Core/Src/libnova_c/elliptic_motion.c \
../Core/Src/libnova_c/heliocentric_time.c \
../Core/Src/libnova_c/hyperbolic_motion.c \
../Core/Src/libnova_c/julian_day.c \
../Core/Src/libnova_c/jupiter.c \
../Core/Src/libnova_c/lunar.c \
../Core/Src/libnova_c/mars.c \
../Core/Src/libnova_c/mercury.c \
../Core/Src/libnova_c/misc.c \
../Core/Src/libnova_c/neptune.c \
../Core/Src/libnova_c/nutation.c \
../Core/Src/libnova_c/parabolic_motion.c \
../Core/Src/libnova_c/parallax.c \
../Core/Src/libnova_c/pluto.c \
../Core/Src/libnova_c/precession.c \
../Core/Src/libnova_c/proper_motion.c \
../Core/Src/libnova_c/refraction.c \
../Core/Src/libnova_c/rise_set.c \
../Core/Src/libnova_c/saturn.c \
../Core/Src/libnova_c/sidereal_time.c \
../Core/Src/libnova_c/solar.c \
../Core/Src/libnova_c/transform.c \
../Core/Src/libnova_c/uranus.c \
../Core/Src/libnova_c/utility.c \
../Core/Src/libnova_c/venus.c \
../Core/Src/libnova_c/vsop87.c 

C_DEPS += \
./Core/Src/libnova_c/aberration.d \
./Core/Src/libnova_c/airmass.d \
./Core/Src/libnova_c/angular_separation.d \
./Core/Src/libnova_c/apparent_position.d \
./Core/Src/libnova_c/asteroid.d \
./Core/Src/libnova_c/comet.d \
./Core/Src/libnova_c/dynamical_time.d \
./Core/Src/libnova_c/earth.d \
./Core/Src/libnova_c/elliptic_motion.d \
./Core/Src/libnova_c/heliocentric_time.d \
./Core/Src/libnova_c/hyperbolic_motion.d \
./Core/Src/libnova_c/julian_day.d \
./Core/Src/libnova_c/jupiter.d \
./Core/Src/libnova_c/lunar.d \
./Core/Src/libnova_c/mars.d \
./Core/Src/libnova_c/mercury.d \
./Core/Src/libnova_c/misc.d \
./Core/Src/libnova_c/neptune.d \
./Core/Src/libnova_c/nutation.d \
./Core/Src/libnova_c/parabolic_motion.d \
./Core/Src/libnova_c/parallax.d \
./Core/Src/libnova_c/pluto.d \
./Core/Src/libnova_c/precession.d \
./Core/Src/libnova_c/proper_motion.d \
./Core/Src/libnova_c/refraction.d \
./Core/Src/libnova_c/rise_set.d \
./Core/Src/libnova_c/saturn.d \
./Core/Src/libnova_c/sidereal_time.d \
./Core/Src/libnova_c/solar.d \
./Core/Src/libnova_c/transform.d \
./Core/Src/libnova_c/uranus.d \
./Core/Src/libnova_c/utility.d \
./Core/Src/libnova_c/venus.d \
./Core/Src/libnova_c/vsop87.d 

OBJS += \
./Core/Src/libnova_c/aberration.o \
./Core/Src/libnova_c/airmass.o \
./Core/Src/libnova_c/angular_separation.o \
./Core/Src/libnova_c/apparent_position.o \
./Core/Src/libnova_c/asteroid.o \
./Core/Src/libnova_c/comet.o \
./Core/Src/libnova_c/dynamical_time.o \
./Core/Src/libnova_c/earth.o \
./Core/Src/libnova_c/elliptic_motion.o \
./Core/Src/libnova_c/heliocentric_time.o \
./Core/Src/libnova_c/hyperbolic_motion.o \
./Core/Src/libnova_c/julian_day.o \
./Core/Src/libnova_c/jupiter.o \
./Core/Src/libnova_c/lunar.o \
./Core/Src/libnova_c/mars.o \
./Core/Src/libnova_c/mercury.o \
./Core/Src/libnova_c/misc.o \
./Core/Src/libnova_c/neptune.o \
./Core/Src/libnova_c/nutation.o \
./Core/Src/libnova_c/parabolic_motion.o \
./Core/Src/libnova_c/parallax.o \
./Core/Src/libnova_c/pluto.o \
./Core/Src/libnova_c/precession.o \
./Core/Src/libnova_c/proper_motion.o \
./Core/Src/libnova_c/refraction.o \
./Core/Src/libnova_c/rise_set.o \
./Core/Src/libnova_c/saturn.o \
./Core/Src/libnova_c/sidereal_time.o \
./Core/Src/libnova_c/solar.o \
./Core/Src/libnova_c/transform.o \
./Core/Src/libnova_c/uranus.o \
./Core/Src/libnova_c/utility.o \
./Core/Src/libnova_c/venus.o \
./Core/Src/libnova_c/vsop87.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/libnova_c/%.o Core/Src/libnova_c/%.su Core/Src/libnova_c/%.cyclo: ../Core/Src/libnova_c/%.c Core/Src/libnova_c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-libnova_c

clean-Core-2f-Src-2f-libnova_c:
	-$(RM) ./Core/Src/libnova_c/aberration.cyclo ./Core/Src/libnova_c/aberration.d ./Core/Src/libnova_c/aberration.o ./Core/Src/libnova_c/aberration.su ./Core/Src/libnova_c/airmass.cyclo ./Core/Src/libnova_c/airmass.d ./Core/Src/libnova_c/airmass.o ./Core/Src/libnova_c/airmass.su ./Core/Src/libnova_c/angular_separation.cyclo ./Core/Src/libnova_c/angular_separation.d ./Core/Src/libnova_c/angular_separation.o ./Core/Src/libnova_c/angular_separation.su ./Core/Src/libnova_c/apparent_position.cyclo ./Core/Src/libnova_c/apparent_position.d ./Core/Src/libnova_c/apparent_position.o ./Core/Src/libnova_c/apparent_position.su ./Core/Src/libnova_c/asteroid.cyclo ./Core/Src/libnova_c/asteroid.d ./Core/Src/libnova_c/asteroid.o ./Core/Src/libnova_c/asteroid.su ./Core/Src/libnova_c/comet.cyclo ./Core/Src/libnova_c/comet.d ./Core/Src/libnova_c/comet.o ./Core/Src/libnova_c/comet.su ./Core/Src/libnova_c/dynamical_time.cyclo ./Core/Src/libnova_c/dynamical_time.d ./Core/Src/libnova_c/dynamical_time.o ./Core/Src/libnova_c/dynamical_time.su ./Core/Src/libnova_c/earth.cyclo ./Core/Src/libnova_c/earth.d ./Core/Src/libnova_c/earth.o ./Core/Src/libnova_c/earth.su ./Core/Src/libnova_c/elliptic_motion.cyclo ./Core/Src/libnova_c/elliptic_motion.d ./Core/Src/libnova_c/elliptic_motion.o ./Core/Src/libnova_c/elliptic_motion.su ./Core/Src/libnova_c/heliocentric_time.cyclo ./Core/Src/libnova_c/heliocentric_time.d ./Core/Src/libnova_c/heliocentric_time.o ./Core/Src/libnova_c/heliocentric_time.su ./Core/Src/libnova_c/hyperbolic_motion.cyclo ./Core/Src/libnova_c/hyperbolic_motion.d ./Core/Src/libnova_c/hyperbolic_motion.o ./Core/Src/libnova_c/hyperbolic_motion.su ./Core/Src/libnova_c/julian_day.cyclo ./Core/Src/libnova_c/julian_day.d ./Core/Src/libnova_c/julian_day.o ./Core/Src/libnova_c/julian_day.su ./Core/Src/libnova_c/jupiter.cyclo ./Core/Src/libnova_c/jupiter.d ./Core/Src/libnova_c/jupiter.o ./Core/Src/libnova_c/jupiter.su ./Core/Src/libnova_c/lunar.cyclo ./Core/Src/libnova_c/lunar.d ./Core/Src/libnova_c/lunar.o ./Core/Src/libnova_c/lunar.su ./Core/Src/libnova_c/mars.cyclo ./Core/Src/libnova_c/mars.d ./Core/Src/libnova_c/mars.o ./Core/Src/libnova_c/mars.su ./Core/Src/libnova_c/mercury.cyclo ./Core/Src/libnova_c/mercury.d ./Core/Src/libnova_c/mercury.o ./Core/Src/libnova_c/mercury.su ./Core/Src/libnova_c/misc.cyclo ./Core/Src/libnova_c/misc.d ./Core/Src/libnova_c/misc.o ./Core/Src/libnova_c/misc.su ./Core/Src/libnova_c/neptune.cyclo ./Core/Src/libnova_c/neptune.d ./Core/Src/libnova_c/neptune.o ./Core/Src/libnova_c/neptune.su ./Core/Src/libnova_c/nutation.cyclo ./Core/Src/libnova_c/nutation.d ./Core/Src/libnova_c/nutation.o ./Core/Src/libnova_c/nutation.su ./Core/Src/libnova_c/parabolic_motion.cyclo ./Core/Src/libnova_c/parabolic_motion.d ./Core/Src/libnova_c/parabolic_motion.o ./Core/Src/libnova_c/parabolic_motion.su ./Core/Src/libnova_c/parallax.cyclo ./Core/Src/libnova_c/parallax.d ./Core/Src/libnova_c/parallax.o ./Core/Src/libnova_c/parallax.su ./Core/Src/libnova_c/pluto.cyclo ./Core/Src/libnova_c/pluto.d ./Core/Src/libnova_c/pluto.o ./Core/Src/libnova_c/pluto.su ./Core/Src/libnova_c/precession.cyclo ./Core/Src/libnova_c/precession.d ./Core/Src/libnova_c/precession.o ./Core/Src/libnova_c/precession.su ./Core/Src/libnova_c/proper_motion.cyclo ./Core/Src/libnova_c/proper_motion.d ./Core/Src/libnova_c/proper_motion.o ./Core/Src/libnova_c/proper_motion.su ./Core/Src/libnova_c/refraction.cyclo ./Core/Src/libnova_c/refraction.d ./Core/Src/libnova_c/refraction.o ./Core/Src/libnova_c/refraction.su ./Core/Src/libnova_c/rise_set.cyclo ./Core/Src/libnova_c/rise_set.d ./Core/Src/libnova_c/rise_set.o ./Core/Src/libnova_c/rise_set.su ./Core/Src/libnova_c/saturn.cyclo ./Core/Src/libnova_c/saturn.d ./Core/Src/libnova_c/saturn.o ./Core/Src/libnova_c/saturn.su ./Core/Src/libnova_c/sidereal_time.cyclo ./Core/Src/libnova_c/sidereal_time.d ./Core/Src/libnova_c/sidereal_time.o ./Core/Src/libnova_c/sidereal_time.su ./Core/Src/libnova_c/solar.cyclo ./Core/Src/libnova_c/solar.d ./Core/Src/libnova_c/solar.o ./Core/Src/libnova_c/solar.su ./Core/Src/libnova_c/transform.cyclo ./Core/Src/libnova_c/transform.d ./Core/Src/libnova_c/transform.o ./Core/Src/libnova_c/transform.su ./Core/Src/libnova_c/uranus.cyclo ./Core/Src/libnova_c/uranus.d ./Core/Src/libnova_c/uranus.o ./Core/Src/libnova_c/uranus.su ./Core/Src/libnova_c/utility.cyclo ./Core/Src/libnova_c/utility.d ./Core/Src/libnova_c/utility.o ./Core/Src/libnova_c/utility.su ./Core/Src/libnova_c/venus.cyclo ./Core/Src/libnova_c/venus.d ./Core/Src/libnova_c/venus.o ./Core/Src/libnova_c/venus.su ./Core/Src/libnova_c/vsop87.cyclo ./Core/Src/libnova_c/vsop87.d ./Core/Src/libnova_c/vsop87.o ./Core/Src/libnova_c/vsop87.su

.PHONY: clean-Core-2f-Src-2f-libnova_c

