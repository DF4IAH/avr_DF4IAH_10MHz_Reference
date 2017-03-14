################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../Src/usbdrv_fw/usbdrvasm.asm 

C_SRCS += \
../Src/usbdrv_fw/oddebug.c \
../Src/usbdrv_fw/usbdrv.c 

S_UPPER_SRCS += \
../Src/usbdrv_fw/usbdrvasm.S 

OBJS += \
./Src/usbdrv_fw/oddebug.o \
./Src/usbdrv_fw/usbdrv.o \
./Src/usbdrv_fw/usbdrvasm.o 

ASM_DEPS += \
./Src/usbdrv_fw/usbdrvasm.d 

S_UPPER_DEPS += \
./Src/usbdrv_fw/usbdrvasm.d 

C_DEPS += \
./Src/usbdrv_fw/oddebug.d \
./Src/usbdrv_fw/usbdrv.d 


# Each subdirectory must supply rules for building sources it contributes
Src/usbdrv_fw/%.o: ../Src/usbdrv_fw/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/home/espero/git/avr_DF4IAH_10MHz_Reference/Src" -I"/home/espero/git/avr_DF4IAH_Bootloader/Src" -DDEBUG -D__AVR_ATmega328P__ -DBOOTSIZE=2048 -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/usbdrv_fw/%.o: ../Src/usbdrv_fw/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -I"/home/espero/git/avr_DF4IAH_10MHz_Reference/Src" -g2 -gstabs -mmcu=atmega328p -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/usbdrv_fw/%.o: ../Src/usbdrv_fw/%.asm
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -I"/home/espero/git/avr_DF4IAH_10MHz_Reference/Src" -g2 -gstabs -mmcu=atmega328p -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


