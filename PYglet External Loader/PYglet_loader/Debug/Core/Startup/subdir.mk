################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g473rbtx.s 

OBJS += \
./Core/Startup/startup_stm32g473rbtx.o 

S_DEPS += \
./Core/Startup/startup_stm32g473rbtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32g473rbtx.o: ../Core/Startup/startup_stm32g473rbtx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32g473rbtx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

