################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/df4iah_fw_anlgComp.c \
../Src/df4iah_fw_clkFastCtr.c \
../Src/df4iah_fw_clkPullPwm.c \
../Src/df4iah_fw_main.c \
../Src/df4iah_fw_memory.c \
../Src/df4iah_fw_ringbuffer.c \
../Src/df4iah_fw_serial.c \
../Src/df4iah_fw_twi.c \
../Src/df4iah_fw_twi_mcp23017.c \
../Src/df4iah_fw_twi_mcp23017_av1624.c \
../Src/df4iah_fw_usb.c 

OBJS += \
./Src/df4iah_fw_anlgComp.o \
./Src/df4iah_fw_clkFastCtr.o \
./Src/df4iah_fw_clkPullPwm.o \
./Src/df4iah_fw_main.o \
./Src/df4iah_fw_memory.o \
./Src/df4iah_fw_ringbuffer.o \
./Src/df4iah_fw_serial.o \
./Src/df4iah_fw_twi.o \
./Src/df4iah_fw_twi_mcp23017.o \
./Src/df4iah_fw_twi_mcp23017_av1624.o \
./Src/df4iah_fw_usb.o 

C_DEPS += \
./Src/df4iah_fw_anlgComp.d \
./Src/df4iah_fw_clkFastCtr.d \
./Src/df4iah_fw_clkPullPwm.d \
./Src/df4iah_fw_main.d \
./Src/df4iah_fw_memory.d \
./Src/df4iah_fw_ringbuffer.d \
./Src/df4iah_fw_serial.d \
./Src/df4iah_fw_twi.d \
./Src/df4iah_fw_twi_mcp23017.d \
./Src/df4iah_fw_twi_mcp23017_av1624.d \
./Src/df4iah_fw_usb.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/Users/espero/Documents/Eclipse_workspace/avr_DF4IAH_10MHz_Reference/Src" -I"/Users/espero/Documents/Eclipse_workspace/avr_DF4IAH_Bootloader/Src" -DRELEASE -D__AVR_ATmega328P__ -DBOOTSIZE=2048 -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


