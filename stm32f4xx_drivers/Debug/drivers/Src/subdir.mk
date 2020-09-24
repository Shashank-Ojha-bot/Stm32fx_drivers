################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f4xx_gpio_driver.c \
../drivers/Src/stm32f4xx_i2c_driver.c \
../drivers/Src/stm32f4xx_spi_driver.c \
../drivers/Src/stm32f4xx_usart_driver.c 

OBJS += \
./drivers/Src/stm32f4xx_gpio_driver.o \
./drivers/Src/stm32f4xx_i2c_driver.o \
./drivers/Src/stm32f4xx_spi_driver.o \
./drivers/Src/stm32f4xx_usart_driver.o 

C_DEPS += \
./drivers/Src/stm32f4xx_gpio_driver.d \
./drivers/Src/stm32f4xx_i2c_driver.d \
./drivers/Src/stm32f4xx_spi_driver.d \
./drivers/Src/stm32f4xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f4xx_gpio_driver.o: ../drivers/Src/stm32f4xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/Study Materials/Embedded-C/My_workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f4xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f4xx_i2c_driver.o: ../drivers/Src/stm32f4xx_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/Study Materials/Embedded-C/My_workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f4xx_i2c_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f4xx_spi_driver.o: ../drivers/Src/stm32f4xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/Study Materials/Embedded-C/My_workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f4xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f4xx_usart_driver.o: ../drivers/Src/stm32f4xx_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/Study Materials/Embedded-C/My_workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f4xx_usart_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

