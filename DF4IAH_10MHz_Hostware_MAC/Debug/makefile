################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: c_hostware_to_DF4IAH_10MHz_Reference_MAC

# Tool invocations
c_hostware_to_DF4IAH_10MHz_Reference_MAC: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MacOS X C Linker'
	gcc -L/opt/local/lib -o "c_hostware_to_DF4IAH_10MHz_Reference_MAC" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) c_hostware_to_DF4IAH_10MHz_Reference_MAC
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
