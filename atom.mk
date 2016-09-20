
LOCAL_PATH := $(call my-dir)

###############################################################################
# Telemetry library.
###############################################################################

include $(CLEAR_VARS)
LOCAL_MODULE := libtelemetry
LOCAL_DESCRIPTION := Telemetry library
LOCAL_CATEGORY_PATH := libs
LOCAL_SRC_FILES := \
	libtelemetry/src/consumer.cpp \
	libtelemetry/src/logger.cpp \
	libtelemetry/src/producer.cpp \
	libtelemetry/src/registrator.cpp \
	libtelemetry/src/variable.cpp \
	libtelemetry/src/shdctx.cpp \
	libtelemetry/src/wrapper.cpp
LOCAL_EXPORT_CXXFLAGS := -std=c++0x
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/libtelemetry/include
LOCAL_LIBRARIES := libulog libshdata libfutils
include $(BUILD_LIBRARY)

###############################################################################
# Telemetry daemon and plugins.
###############################################################################

include $(CLEAR_VARS)
LOCAL_MODULE := telemetryd
LOCAL_DESCRIPTION := Telemetry daemon
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := telemetryd/telemetryd.cpp
LOCAL_LDLIBS := -ldl
LOCAL_LIBRARIES := libtelemetry libulog libpomp
LOCAL_COPY_FILES = telemetryd/50-telemetryd.rc:etc/boxinit.d/
include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)
LOCAL_MODULE := tlm-blackbox
LOCAL_DESCRIPTION := Telemetry plugin to log data in file system
LOCAL_CATEGORY_PATH := telemetry
LOCAL_DESTDIR := usr/lib/tlm-plugins
LOCAL_SRC_FILES := \
	telemetryd/plugins/blackbox/blackbox.cpp \
	telemetryd/plugins/blackbox/logfile.cpp
LOCAL_LIBRARIES := libtelemetry libulog libpomp zlib
LOCAL_CONDITIONAL_LIBRARIES := OPTIONAL:libshs OPTIONAL:libputils
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtlmblackbox
LOCAL_DESCRIPTION := Client library to read telemetry blackbox files
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := \
	telemetryd/plugins/blackbox/libtlmblackbox.cpp
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/telemetryd/plugins/blackbox/include
LOCAL_C_INCLUDES := $(LOCAL_EXPORT_C_INCLUDES)
LOCAL_LIBRARIES := libtelemetry libulog zlib
include $(BUILD_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-gndctrl
LOCAL_DESCRIPTION := Telemetry plugin to log data to a remote ground control
LOCAL_CATEGORY_PATH := telemetry
LOCAL_DESTDIR := usr/lib/tlm-plugins
LOCAL_SRC_FILES := \
	telemetryd/plugins/gndctrl/gndctrl.cpp
LOCAL_LIBRARIES := libtelemetry libulog libpomp libfutils
LOCAL_CONDITIONAL_LIBRARIES := OPTIONAL:libputils
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtlmgndctrl
LOCAL_DESCRIPTION := Client library to connect to telemetry ground control plugin
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := \
	telemetryd/plugins/gndctrl/libtlmgndctrl.cpp
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/telemetryd/plugins/gndctrl/include
LOCAL_C_INCLUDES := $(LOCAL_EXPORT_C_INCLUDES)
LOCAL_LIBRARIES := libtelemetry libulog libpomp
include $(BUILD_LIBRARY)

###############################################################################
# Tools.
###############################################################################

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-shm-dump
LOCAL_DESCRIPTION := Dump contents of shared memory used by telemetry
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := tools/tlm_shm_dump.c
LOCAL_LIBRARIES := libtelemetry
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-gndctrl-client-netcdf
LOCAL_DESCRIPTION := Telemetry ground control client to log data in a netCDF file
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := tools/gndctrl_client_netcdf.cpp
LOCAL_LIBRARIES := libtlmgndctrl libulog libpomp libfutils netcdf-cxx4
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-shm-generator
LOCAL_DESCRIPTION := Generate fake content in shared memory
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := tools/tlm_shm_generator.c
LOCAL_LIBRARIES := libtelemetry libulog libfutils
include $(BUILD_EXECUTABLE)

###############################################################################
# Example code.
###############################################################################

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-producer
LOCAL_DESCRIPTION := Telemetry producer example
LOCAL_CATEGORY_PATH := telemetry/examples
LOCAL_SRC_FILES := examples/producer.cpp
LOCAL_LIBRARIES := libtelemetry libulog
include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)
LOCAL_MODULE := tlm-consumer
LOCAL_DESCRIPTION := Telemetry consumer example
LOCAL_CATEGORY_PATH := telemetry/examples
LOCAL_SRC_FILES := examples/consumer.cpp
LOCAL_LIBRARIES := libtelemetry libulog
include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)
LOCAL_MODULE := tlm-logger
LOCAL_DESCRIPTION := Telemetry logger example
LOCAL_CATEGORY_PATH := telemetry/examples
LOCAL_SRC_FILES := examples/logger.cpp
LOCAL_LIBRARIES := libtelemetry libulog libfutils
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-blackbox-reader
LOCAL_DESCRIPTION := Telemetry blackbox reader example
LOCAL_CATEGORY_PATH := telemetry/examples
LOCAL_SRC_FILES := examples/tlmbreader.cpp
LOCAL_LIBRARIES := libtlmblackbox libulog
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := tlm-data-logger
LOCAL_DESCRIPTION := Telemetry data logger using gnd-ctrl plugin
LOCAL_CATEGORY_PATH := telemetry
LOCAL_SRC_FILES := tools/gndctrl_datalogger.cpp
LOCAL_LIBRARIES := libtlmgndctrl libulog libpomp
include $(BUILD_EXECUTABLE)
