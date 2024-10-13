################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002_Led_Botton.c 

OBJS += \
./Src/002_Led_Botton.o 

C_DEPS += \
./Src/002_Led_Botton.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_G070RB -DSTM32G070RBTx -DSTM32 -DSTM32G0 -c -I../Inc -I"C:/Users/hp/STM32CubeIDE/workspace_1.16.1/stm32G070RB_Driver/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/002_Led_Botton.cyclo ./Src/002_Led_Botton.d ./Src/002_Led_Botton.o ./Src/002_Led_Botton.su

.PHONY: clean-Src

