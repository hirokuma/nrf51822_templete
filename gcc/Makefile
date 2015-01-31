PROJECT_NAME := ble_ios
SDK_PATH = ../../..
PRJ_PATH = ..
#BOARD_NAME = BOARD_PCA10028
NRF51_TYPE = s110_xxaa

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

TEMPLATE_PATH = $(SDK_PATH)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(SDK_PATH)/components/libraries/button/app_button.c \
$(SDK_PATH)/components/libraries/util/app_error.c \
$(SDK_PATH)/components/libraries/timer/app_timer.c \
$(SDK_PATH)/components/libraries/gpiote/app_gpiote.c \
$(SDK_PATH)/components/libraries/scheduler/app_scheduler.c \
$(SDK_PATH)/components/libraries/util/nrf_assert.c \
$(SDK_PATH)/components/libraries/trace/app_trace.c \
$(SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(SDK_PATH)/components/toolchain/system_nrf51.c \
$(SDK_PATH)/components/ble/common/ble_conn_params.c \
$(SDK_PATH)/components/ble/common/ble_advdata.c \
$(SDK_PATH)/components/ble/common/ble_srv_common.c \
$(PRJ_PATH)/ble_ios.c \
$(PRJ_PATH)/main.c

#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS  = -I$(PRJ_PATH)/config
INC_PATHS += -I../../../../../bsp
INC_PATHS += -I$(SDK_PATH)/components/toolchain/gcc
INC_PATHS += -I$(SDK_PATH)/components/toolchain
INC_PATHS += -I$(SDK_PATH)/components/libraries/button
INC_PATHS += -I$(SDK_PATH)/components/libraries/sensorsim
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_hrs
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_dis
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s110/headers
INC_PATHS += -I$(SDK_PATH)/components/ble/common
INC_PATHS += -I$(SDK_PATH)/components/libraries/timer
INC_PATHS += -I$(SDK_PATH)/components/libraries/gpiote
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_PATH)/components/ble/device_manager
INC_PATHS += -I$(SDK_PATH)/components/ble/device_manager/config
INC_PATHS += -I$(SDK_PATH)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_PATH)/components/libraries/scheduler
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/pstorage/config
INC_PATHS += -I$(SDK_PATH)/components/libraries/util
INC_PATHS += -I$(SDK_PATH)/components/libraries/trace
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_PATH)/components/libraries/fifo

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF51
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DS110
CFLAGS += -DSOFTDEVICE_PRESENT
#CFLAGS += -D$(BOARD_NAME)
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -flto -fno-builtin

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF51
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DS110
ASMFLAGS += -DSOFTDEVICE_PRESENT
#ASMFLAGS += -D$(BOARD_NAME)
#default target - first one defined
default: clean debug

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e debug

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	debug release


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

#debug: CFLAGS += -DDEBUG
debug: CFLAGS += -g3 -O0
debug: ASMFLAGS += -DDEBUG -g3 -O0
debug: OUTPUT_FILENAME := nrf51822_qfaa_s110d
debug: LINKER_SCRIPT=$(SDK_PATH)/components/toolchain/gcc/gcc_nrf51_$(NRF51_TYPE).ld
debug: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo [DEBUG]Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

release: CFLAGS += -DNDEBUG -O3
release: ASMFLAGS += -DNDEBUG -O3
release: OUTPUT_FILENAME := nrf51822_qfaa_s110
release: LINKER_SCRIPT=$(SDK_PATH)/components/toolchain/gcc/gcc_nrf51_$(NRF51_TYPE).ld
release: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo [RELEASE]Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling C file: $(notdir $<): $(CFLAGS)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling ASM file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex:
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --reset --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex
