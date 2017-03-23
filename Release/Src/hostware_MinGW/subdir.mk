################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/hostware_MinGW/main.c \
../Src/hostware_MinGW/opendevice.c \
../Src/hostware_MinGW/terminal.c 

OBJS += \
./Src/hostware_MinGW/main.o \
./Src/hostware_MinGW/opendevice.o \
./Src/hostware_MinGW/terminal.o 

C_DEPS += \
./Src/hostware_MinGW/main.d \
./Src/hostware_MinGW/opendevice.d \
./Src/hostware_MinGW/terminal.d 


# Each subdirectory must supply rules for building sources it contributes
Src/hostware_MinGW/%.o: ../Src/hostware_MinGW/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/Users/espero/Documents/Eclipse_workspace/avr_DF4IAH_10MHz_Reference/Src" -I"/Users/espero/Documents/Eclipse_workspace/avr_DF4IAH_Bootloader/Src" -DRELEASE -D__AVR_ATmega328P__ -DBOOTSIZE=2048 -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


