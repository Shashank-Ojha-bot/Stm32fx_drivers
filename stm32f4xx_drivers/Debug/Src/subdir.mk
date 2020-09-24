################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002.c 

OBJS += \
./Src/002.o 

C_DEPS += \
./Src/002.d 


# Each subdirectory must supply rules for building sources it contributes
Src/002.o: ../Src/002.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/Study Materials/Embedded-C/My_workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/002.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

