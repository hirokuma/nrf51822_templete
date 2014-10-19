OUTPUT_FILENAME := out

TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

# application source
C_SOURCE_FILES += main.c
C_SOURCE_FILES += ble_ios.c

C_SOURCE_FILES += softdevice_handler.c

C_SOURCE_FILES += app_scheduler.c
C_SOURCE_FILES += app_button.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += app_timer.c
#C_SOURCE_FILES += pstorage.c
#C_SOURCE_FILES += crc16.c

C_SOURCE_FILES += ble_srv_common.c
#C_SOURCE_FILES += ble_sensorsim.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_debug_assert_handler.c
#C_SOURCE_FILES += ble_error_log.c
C_SOURCE_FILES += ble_conn_params.c

#SDK_PATH = C:/NORDIC~1/NRF51S~1.0/Nordic/nrf51822/
#SDK_PATH = E:/Home/Hiroshi/Work/Bluetooth/03_Nordic/nrf51_sdk_v6_1_0_b2ec2e6/nrf51822/
SDK_PATH = $(NORDIC_NRF51)/

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

CFLAGS := -DBLE_STACK_SUPPORT_REQD

#user application debug
#debug:    CFLAGS += -DDEBUG_NRF_USER

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections

INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

include $(SDK_PATH)Source/templates/gcc/Makefile.common
