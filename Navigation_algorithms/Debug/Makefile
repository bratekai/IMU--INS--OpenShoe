################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := -lnewlib_addons-at32ucr3-speed_opt 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
LIB_AS_ARGS :=
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
HEX_FLASH_FILE_PATH :=
HEX_FLASH_FILE_PATH_AS_ARGS :=
HEX_EEPROM_FILE_PATH :=
HEX_EEPROM_FILE_PATH_AS_ARGS :=
LSS_FILE_PATH :=
LSS_FILE_PATH_AS_ARGS :=
MAP_FILE_PATH :=
MAP_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=C:/Program Files (x86)/Atmel/AVR Studio 5.0/AVR ToolChain/bin/
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../src \
../src/asf/ \
../src/asf/avr32/ \
../src/asf/avr32/drivers/ \
../src/asf/avr32/drivers/intc/ \
../src/asf/avr32/utils/ \
../src/asf/avr32/utils/debug/ \
../src/asf/avr32/utils/preprocessor/ \
../src/asf/avr32/utils/startup/ \
../src/asf/common/ \
../src/asf/common/boards/ \
../src/asf/common/utils/ \
../src/asf/common/utils/interrupt/ \
../src/asf/thirdparty/ \
../src/asf/thirdparty/newlib_addons/ \
../src/asf/thirdparty/newlib_addons/libs/ \
../src/asf/thirdparty/newlib_addons/libs/at32ucr3/ \
../src/asf/thirdparty/newlib_addons/libs/include/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../src/asf/avr32/drivers/intc/intc.c \
../src/asf/avr32/utils/debug/debug.c \
../src/nav_eq.c


PREPROCESSING_SRCS +=  \
../src/asf/avr32/drivers/intc/exception.S \
../src/asf/avr32/utils/startup/startup_uc3.S \
../src/asf/avr32/utils/startup/trampoline_uc3.S


ASM_SRCS += 


OBJS +=  \
src/asf/avr32/drivers/intc/exception.o \
src/asf/avr32/drivers/intc/intc.o \
src/asf/avr32/utils/debug/debug.o \
src/asf/avr32/utils/startup/startup_uc3.o \
src/asf/avr32/utils/startup/trampoline_uc3.o \
src/nav_eq.o


OBJS_AS_ARGS +=  \
src/asf/avr32/drivers/intc/exception.o \
src/asf/avr32/drivers/intc/intc.o \
src/asf/avr32/utils/debug/debug.o \
src/asf/avr32/utils/startup/startup_uc3.o \
src/asf/avr32/utils/startup/trampoline_uc3.o \
src/nav_eq.o


C_DEPS +=  \
src/asf/avr32/drivers/intc/intc.d \
src/asf/avr32/utils/debug/debug.d \
src/nav_eq.d


C_DEPS_AS_ARGS +=  \
src/asf/avr32/drivers/intc/intc.d \
src/asf/avr32/utils/debug/debug.d \
src/nav_eq.d


OUTPUT_FILE_PATH +=libNavigation_algorithms.a

OUTPUT_FILE_PATH_AS_ARGS +=libNavigation_algorithms.a

HEX_FLASH_FILE_PATH +=Navigation_algorithms.hex

HEX_FLASH_FILE_PATH_AS_ARGS +=Navigation_algorithms.hex

HEX_EEPROM_FILE_PATH +=Navigation_algorithms.eep

HEX_EEPROM_FILE_PATH_AS_ARGS +=Navigation_algorithms.eep

LSS_FILE_PATH +=Navigation_algorithms.lss

LSS_FILE_PATH_AS_ARGS +=Navigation_algorithms.lss

MAP_FILE_PATH =Navigation_algorithms.map

MAP_FILE_PATH_AS_ARGS =Navigation_algorithms.map

LIB_AS_ARGS +=libNavigation_algorithms.a

ADDITIONAL_DEPENDENCIES:= $(HEX_FLASH_FILE_PATH) $(LSS_FILE_PATH) $(HEX_EEPROM_FILE_PATH) size

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler







src/asf/avr32/drivers/intc/%.o: ../src/asf/avr32/drivers/intc/%.c
	@echo Building file: $<
	@echo Invoking: AVR32/GNU C Compiler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -I"../src/asf/thirdparty/newlib_addons/libs/include" -I"../src/asf/common/utils" -I"../src/asf/avr32/drivers/intc" -I"../src/asf/avr32/utils" -I"../src/asf/avr32/utils/preprocessor" -I"../src/asf/common/boards" -I"../src/asf/avr32/utils/debug"  -O1 -ffunction-sections -masm-addr-pseudos -g3 -Wall -c -std=gnu99 -mhard-float  -mpart=uc3c2512c   -MD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo Finished building: $<

src/asf/avr32/utils/debug/%.o: ../src/asf/avr32/utils/debug/%.c
	@echo Building file: $<
	@echo Invoking: AVR32/GNU C Compiler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -I"../src/asf/thirdparty/newlib_addons/libs/include" -I"../src/asf/common/utils" -I"../src/asf/avr32/drivers/intc" -I"../src/asf/avr32/utils" -I"../src/asf/avr32/utils/preprocessor" -I"../src/asf/common/boards" -I"../src/asf/avr32/utils/debug"  -O1 -ffunction-sections -masm-addr-pseudos -g3 -Wall -c -std=gnu99 -mhard-float  -mpart=uc3c2512c   -MD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo Finished building: $<

src/%.o: ../src/%.c
	@echo Building file: $<
	@echo Invoking: AVR32/GNU C Compiler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -I"../src/asf/thirdparty/newlib_addons/libs/include" -I"../src/asf/common/utils" -I"../src/asf/avr32/drivers/intc" -I"../src/asf/avr32/utils" -I"../src/asf/avr32/utils/preprocessor" -I"../src/asf/common/boards" -I"../src/asf/avr32/utils/debug"  -O1 -ffunction-sections -masm-addr-pseudos -g3 -Wall -c -std=gnu99 -mhard-float  -mpart=uc3c2512c   -MD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo Finished building: $<



# AVR32/GNU Preprocessing Assembler

src/asf/avr32/drivers/intc/%.o: ../src/asf/avr32/drivers/intc/%.x
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -I "../src/asf/thirdparty/newlib_addons/libs/include" -I "../src/asf/common/utils" -I "../src/asf/avr32/drivers/intc" -I "../src/asf/avr32/utils" -I "../src/asf/avr32/utils/preprocessor" -I "../src/asf/common/boards" -I "../src/asf/avr32/utils/debug"  -g3  -mpart=uc3c2512c   -o"$@" "$<"
	@echo Finished building: $<

src/asf/avr32/drivers/intc/%.o: ../src/asf/avr32/drivers/intc/%.S
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -I "../src/asf/thirdparty/newlib_addons/libs/include" -I "../src/asf/common/utils" -I "../src/asf/avr32/drivers/intc" -I "../src/asf/avr32/utils" -I "../src/asf/avr32/utils/preprocessor" -I "../src/asf/common/boards" -I "../src/asf/avr32/utils/debug"  -g3  -mpart=uc3c2512c   -o"$@" "$<"
	@echo Finished building: $<

src/asf/avr32/utils/startup/%.o: ../src/asf/avr32/utils/startup/%.x
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -I "../src/asf/thirdparty/newlib_addons/libs/include" -I "../src/asf/common/utils" -I "../src/asf/avr32/drivers/intc" -I "../src/asf/avr32/utils" -I "../src/asf/avr32/utils/preprocessor" -I "../src/asf/common/boards" -I "../src/asf/avr32/utils/debug"  -g3  -mpart=uc3c2512c   -o"$@" "$<"
	@echo Finished building: $<

src/asf/avr32/utils/startup/%.o: ../src/asf/avr32/utils/startup/%.S
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler
	$(QUOTE)$(AVR_APP_PATH)avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -I "../src/asf/thirdparty/newlib_addons/libs/include" -I "../src/asf/common/utils" -I "../src/asf/avr32/drivers/intc" -I "../src/asf/avr32/utils" -I "../src/asf/avr32/utils/preprocessor" -I "../src/asf/common/boards" -I "../src/asf/avr32/utils/debug"  -g3  -mpart=uc3c2512c   -o"$@" "$<"
	@echo Finished building: $<



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

# AVR32/GNU C Archiver
$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo	Building target: $@
	@echo	Invoking: AVR32/GNU C Archiver 
	$(QUOTE)$(AVR_APP_PATH)avr32-ar.exe$(QUOTE) -r  $(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS)
	@echo	Finished building target: $@



$(HEX_FLASH_FILE_PATH): $(OUTPUT_FILE_PATH)
	

$(HEX_EEPROM_FILE_PATH): $(OUTPUT_FILE_PATH)
	

$(LSS_FILE_PATH): $(OUTPUT_FILE_PATH)
	

size: $(OUTPUT_FILE_PATH)
	

# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) $(LIB_AS_ARGS) $(HEX_FLASH_FILE_PATH_AS_ARGS) $(HEX_EEPROM_FILE_PATH_AS_ARGS) $(LSS_FILE_PATH_AS_ARGS) $(MAP_FILE_PATH_AS_ARGS)

