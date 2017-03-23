################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/hostware_MAC/main.c \
../Src/hostware_MAC/opendevice.c \
../Src/hostware_MAC/terminal.c 

OBJS += \
./Src/hostware_MAC/main.o \
./Src/hostware_MAC/opendevice.o \
./Src/hostware_MAC/terminal.o 

C_DEPS += \
./Src/hostware_MAC/main.d \
./Src/hostware_MAC/opendevice.d \
./Src/hostware_MAC/terminal.d 


# Each subdirectory must supply rules for building sources it contributes
Src/hostware_MAC/%.o: ../Src/hostware_MAC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/Users/espero/Documents/Eclipse_workspace/avr_DF4IAH_10MHz_Reference/Src" -I"/Users/espero/Documents/Eclipse_workspace/avr_DF4IAH_Bootloader/Src" -DRELEASE -D__AVR_ATmega328P__ -DBOOTSIZE=2048 -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


