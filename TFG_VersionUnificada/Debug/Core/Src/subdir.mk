################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ESP_DATA_HANDLER.c \
../Core/Src/UartRingbuffer.c \
../Core/Src/main.c \
../Core/Src/mqtt_raspberry.c \
../Core/Src/sht85.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/veml7700.c \
../Core/Src/wifi_thingspeak.c 

OBJS += \
./Core/Src/ESP_DATA_HANDLER.o \
./Core/Src/UartRingbuffer.o \
./Core/Src/main.o \
./Core/Src/mqtt_raspberry.o \
./Core/Src/sht85.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/veml7700.o \
./Core/Src/wifi_thingspeak.o 

C_DEPS += \
./Core/Src/ESP_DATA_HANDLER.d \
./Core/Src/UartRingbuffer.d \
./Core/Src/main.d \
./Core/Src/mqtt_raspberry.d \
./Core/Src/sht85.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/veml7700.d \
./Core/Src/wifi_thingspeak.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ESP_DATA_HANDLER.cyclo ./Core/Src/ESP_DATA_HANDLER.d ./Core/Src/ESP_DATA_HANDLER.o ./Core/Src/ESP_DATA_HANDLER.su ./Core/Src/UartRingbuffer.cyclo ./Core/Src/UartRingbuffer.d ./Core/Src/UartRingbuffer.o ./Core/Src/UartRingbuffer.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mqtt_raspberry.cyclo ./Core/Src/mqtt_raspberry.d ./Core/Src/mqtt_raspberry.o ./Core/Src/mqtt_raspberry.su ./Core/Src/sht85.cyclo ./Core/Src/sht85.d ./Core/Src/sht85.o ./Core/Src/sht85.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/veml7700.cyclo ./Core/Src/veml7700.d ./Core/Src/veml7700.o ./Core/Src/veml7700.su ./Core/Src/wifi_thingspeak.cyclo ./Core/Src/wifi_thingspeak.d ./Core/Src/wifi_thingspeak.o ./Core/Src/wifi_thingspeak.su

.PHONY: clean-Core-2f-Src

