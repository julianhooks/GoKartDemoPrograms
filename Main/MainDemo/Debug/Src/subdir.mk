################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/app.c \
../Src/av_com.c \
../Src/main.c \
../Src/spektrum.c \
../Src/spektrum_nucelo.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f4xx.c \
../Src/uart_serial.c \
../Src/utils.c 

OBJS += \
./Src/app.o \
./Src/av_com.o \
./Src/main.o \
./Src/spektrum.o \
./Src/spektrum_nucelo.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f4xx.o \
./Src/uart_serial.o \
./Src/utils.o 

C_DEPS += \
./Src/app.d \
./Src/av_com.d \
./Src/main.d \
./Src/spektrum.d \
./Src/spektrum_nucelo.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f4xx.d \
./Src/uart_serial.d \
./Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F439xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/app.cyclo ./Src/app.d ./Src/app.o ./Src/app.su ./Src/av_com.cyclo ./Src/av_com.d ./Src/av_com.o ./Src/av_com.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/spektrum.cyclo ./Src/spektrum.d ./Src/spektrum.o ./Src/spektrum.su ./Src/spektrum_nucelo.cyclo ./Src/spektrum_nucelo.d ./Src/spektrum_nucelo.o ./Src/spektrum_nucelo.su ./Src/stm32f4xx_hal_msp.cyclo ./Src/stm32f4xx_hal_msp.d ./Src/stm32f4xx_hal_msp.o ./Src/stm32f4xx_hal_msp.su ./Src/stm32f4xx_it.cyclo ./Src/stm32f4xx_it.d ./Src/stm32f4xx_it.o ./Src/stm32f4xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f4xx.cyclo ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su ./Src/uart_serial.cyclo ./Src/uart_serial.d ./Src/uart_serial.o ./Src/uart_serial.su ./Src/utils.cyclo ./Src/utils.d ./Src/utils.o ./Src/utils.su

.PHONY: clean-Src

