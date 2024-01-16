################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DHT/dht.c 

OBJS += \
./Drivers/DHT/dht.o 

C_DEPS += \
./Drivers/DHT/dht.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DHT/%.o Drivers/DHT/%.su Drivers/DHT/%.cyclo: ../Drivers/DHT/%.c Drivers/DHT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kai.dietrich/fancy/Drivers/DHT" -I"C:/Users/kai.dietrich/fancy/Drivers/tm1637" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-DHT

clean-Drivers-2f-DHT:
	-$(RM) ./Drivers/DHT/dht.cyclo ./Drivers/DHT/dht.d ./Drivers/DHT/dht.o ./Drivers/DHT/dht.su

.PHONY: clean-Drivers-2f-DHT

