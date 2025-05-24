################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/L6474.cpp \
../Core/Src/Pwm.cpp \
../Core/Src/driver.cpp 

C_SRCS += \
../Core/Src/EtherShield.c \
../Core/Src/dhcp.c \
../Core/Src/dnslkup.c \
../Core/Src/enc28j60.c \
../Core/Src/error_handler.c \
../Core/Src/ip_arp_udp_tcp.c \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/websrv_help_functions.c 

C_DEPS += \
./Core/Src/EtherShield.d \
./Core/Src/dhcp.d \
./Core/Src/dnslkup.d \
./Core/Src/enc28j60.d \
./Core/Src/error_handler.d \
./Core/Src/ip_arp_udp_tcp.d \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/websrv_help_functions.d 

OBJS += \
./Core/Src/EtherShield.o \
./Core/Src/L6474.o \
./Core/Src/Pwm.o \
./Core/Src/dhcp.o \
./Core/Src/dnslkup.o \
./Core/Src/driver.o \
./Core/Src/enc28j60.o \
./Core/Src/error_handler.o \
./Core/Src/ip_arp_udp_tcp.o \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/websrv_help_functions.o 

CPP_DEPS += \
./Core/Src/L6474.d \
./Core/Src/Pwm.d \
./Core/Src/driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Core/Inc/ENC28J60 -I../Core/Inc/L6474 -I../Core/Inc/Common -I../Core/Inc/Actuators -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Core/Inc/ENC28J60 -I../Core/Inc/Actuators -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Core/Inc/Common -I../Core/Inc/L6474 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/EtherShield.cyclo ./Core/Src/EtherShield.d ./Core/Src/EtherShield.o ./Core/Src/EtherShield.su ./Core/Src/L6474.cyclo ./Core/Src/L6474.d ./Core/Src/L6474.o ./Core/Src/L6474.su ./Core/Src/Pwm.cyclo ./Core/Src/Pwm.d ./Core/Src/Pwm.o ./Core/Src/Pwm.su ./Core/Src/dhcp.cyclo ./Core/Src/dhcp.d ./Core/Src/dhcp.o ./Core/Src/dhcp.su ./Core/Src/dnslkup.cyclo ./Core/Src/dnslkup.d ./Core/Src/dnslkup.o ./Core/Src/dnslkup.su ./Core/Src/driver.cyclo ./Core/Src/driver.d ./Core/Src/driver.o ./Core/Src/driver.su ./Core/Src/enc28j60.cyclo ./Core/Src/enc28j60.d ./Core/Src/enc28j60.o ./Core/Src/enc28j60.su ./Core/Src/error_handler.cyclo ./Core/Src/error_handler.d ./Core/Src/error_handler.o ./Core/Src/error_handler.su ./Core/Src/ip_arp_udp_tcp.cyclo ./Core/Src/ip_arp_udp_tcp.d ./Core/Src/ip_arp_udp_tcp.o ./Core/Src/ip_arp_udp_tcp.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/websrv_help_functions.cyclo ./Core/Src/websrv_help_functions.d ./Core/Src/websrv_help_functions.o ./Core/Src/websrv_help_functions.su

.PHONY: clean-Core-2f-Src

