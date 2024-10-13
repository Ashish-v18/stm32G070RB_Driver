################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32g0xx_gpio_driver.c 

OBJS += \
./driver/Src/stm32g0xx_gpio_driver.o 

C_DEPS += \
./driver/Src/stm32g0xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o driver/Src/%.su driver/Src/%.cyclo: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_G070RB -DSTM32G070RBTx -DSTM32 -DSTM32G0 -c -I../Inc -I"C:/Users/hp/STM32CubeIDE/workspace_1.16.1/stm32G070RB_Driver/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-driver-2f-Src

clean-driver-2f-Src:
	-$(RM) ./driver/Src/stm32g0xx_gpio_driver.cyclo ./driver/Src/stm32g0xx_gpio_driver.d ./driver/Src/stm32g0xx_gpio_driver.o ./driver/Src/stm32g0xx_gpio_driver.su

.PHONY: clean-driver-2f-Src

