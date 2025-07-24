MCU := F421
PART := AT32F421K8U7

MCU_LC := $(call lc,$(MCU))

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU_LC)

MCU_$(MCU) := -mcpu=cortex-m4 -mthumb
LDSCRIPT_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.ld)

SRC_BASE_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Startup \
	$(HAL_FOLDER_$(MCU))/Drivers/drivers/src

SRC_DIR_$(MCU) := $(SRC_BASE_DIR_$(MCU)) \
	$(HAL_FOLDER_$(MCU))/Src \
	$(MAIN_SRC_DIR)/Sport/src

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/drivers/inc \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/cm4/core_support \
	-I$(HAL_FOLDER_$(MCU))/Drivers/CMSIS/cm4/device_support \
	-I$(MAIN_SRC_DIR)/Sport

CFLAGS_$(MCU) += \
	 -D$(PART) \
	 -DUSE_STDPERIPH_DRIVER \
	 -DENABLE_FRSKY_SPORT_TELEMETRY

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.[cs]))
