#/*
# *  RPLIDAR SDK
# *
# *  Copyright (c) 2009 - 2014 RoboPeak Team
# *  http://www.robopeak.com
# *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
# *  http://www.slamtec.com
# *
# */
#/*
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * 1. Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# * 2. Redistributions in binary form must reproduce the above copyright notice,
# *    this list of conditions and the following disclaimer in the documentation
# *    and/or other materials provided with the distribution.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
# */

#
#  Common Definition for RoboPeak Building
#
#
#

HOME_TREE:=$(realpath $(HOME_TREE))

BUILD_ROOT  = $(HOME_TREE)/build

#valid platforms are:
#Linux Darwin

#valid architectures are:
#x86

BUILD_HOST_PLATFORM := $(shell uname -s)
BUILD_HOST_ARCH := $(shell uname -m)

ifeq ($(BUILD_HOST_ARCH),x86_64)
BUILD_HOST_ARCH:=x86
BUILD_HOST_HW_PLATFORM := pc
CFLAGS +=
CXXFLAGS +=
endif

ifeq ($(BUILD_HOST_ARCH),armv7l)
BUILD_HOST_ARCH:=arm
BUILD_HOST_HW_PLATFORM := raspi
CFLAGS +=
CXXFLAGS +=
endif

BUILD_TARGET_ARCH?=$(BUILD_HOST_ARCH)
BUILD_TARGET_PLATFORM?=$(BUILD_HOST_PLATFORM)
BUILD_TARGET_HW_PLATFORM?=$(BUILD_HOST_HW_PLATFORM)

RM = rm -f
MV = mv -f
MKDIR = mkdir -p
RMDIR = rm -rf

ifeq ($(V),1)
CMD_HIDE:=
else
CMD_HIDE:=@
endif

ifeq ($(CMD_HIDE),@)
CMD_HIDE_CC="@echo   CC   $^; @"
CMD_HIDE_CXX="@echo  CXX  $^; @"
CMD_HIDE_LD="@echo   LD   $@; @"
endif

ifdef DEBUG
OUTPUT_BUILD_PREFIX =Debug
else
OUTPUT_BUILD_PREFIX =Release
endif

BUILD_OUTPUT_ROOT = $(BUILD_ROOT)/output/$(BUILD_TARGET_PLATFORM)/$(OUTPUT_BUILD_PREFIX)
BUILD_OBJ_ROOT    = $(BUILD_ROOT)/obj/$(BUILD_TARGET_PLATFORM)/$(OUTPUT_BUILD_PREFIX)

SDK_LIB_CORENAME := rplidar_sdk
SDK_STATIC_MODULE:=lib$(SDK_LIB_CORENAME).a
TARGET_OBJ_ROOT = $(BUILD_OBJ_ROOT)/$(MODULE_NAME)
TARGET_SRC_ROOT = $(HOME_TREE)/src
#SDK_TARGET = $(BUILD_OUTPUT_ROOT)/$(SDK_STATIC_MODULE)

# rplidar lib import
ifeq ($(BUILD_TARGET_ARCH),arm)
RPLIDAR_SDK_IMPORT = $(HOME_TREE)/import/rplidar_sdk2018/lib/$(BUILD_TARGET_HW_PLATFORM)/$(SDK_STATIC_MODULE)
endif
ifeq ($(BUILD_TARGET_ARCH),x86)
RPLIDAR_SDK_IMPORT = $(HOME_TREE)/import/rplidar_sdk2018/lib/x86/$(SDK_STATIC_MODULE)
endif

# yaml-cpp lib import
ifeq ($(YAML),import)
ifeq ($(BUILD_TARGET_ARCH),arm)
YAML_IMPORT = $(HOME_TREE)/import/yaml/lib/$(BUILD_TARGET_HW_PLATFORM)/libyaml-cpp.a
endif
ifeq ($(BUILD_TARGET_ARCH),x86)
YAML_IMPORT = $(HOME_TREE)/import/yaml/lib/x86/$(BUILD_HOST_PLATFORM)/libyaml-cpp.a
endif
endif

APP_TARGET=$(BUILD_OUTPUT_ROOT)/$(MODULE_NAME)

# Debugging format.
DEBUG_TYPE = gdb

#Optimize Level
OPT = 2 

# Place -D or -U options here
ifeq ($(BUILD_TARGET_ARCH),x86)
CDEFS += -D_CPU_ENDIAN_SMALL
endif

#ifeq ($(BUILD_TARGET_PLATFORM),Darwin)
#CDEFS += -D_MACOS
#else
#LD_LIBS += -lrt
#endif


CDEFS += $(EXTRA_DEFS)

CXXDEFS +=
CXXDEFS += $(CDEFS) $(EXTRA_DEFS)

C_INCLUDES += -I$(HOME_TREE)/src
C_INCLUDES += -I$(HOME_TREE)/import/rplidar_sdk2018/include
ifeq ($(YAML),import)
C_INCLUDES += -I$(HOME_TREE)/import/yaml/include
endif

# -I/opt/gumstix/include 
# -I/usr/include/newlib -I/usr/include/newlib/c++/4.8 -I/usr/include/newlib/c++/4.8/arm-none-eabi 
CXX_INCLUDES += $(C_INCLUDES) $(EXTRA_CXXINC)

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG_TYPE)
CWARN = -Wall 
CTUNING = -funsigned-char 

# force everything to become PIC, unless explicitly claimed
ifndef BUILD_APP_NO_PIC_CODE
CTUNING += -fPIC
endif

ifdef DEBUG
OPT_FLAG = $(CDEBUG)
CDEFS += -D_DEBUG -DDEBUG
else
OPT_FLAG = -O$(OPT)
CDEFS += -DNDEBUG -D_NDEBUG
endif

CXXEXTRA = -std=c++11 -Wno-deprecated

CFLAGS += -ggdb $(OPT_FLAG) $(CDEFS) $(C_INCLUDES) $(CWARN) $(CSTANDARD) $(CEXTRA) $(CTUNING) -Wstrict-prototypes

CXXFLAGS += -ggdb $(OPT_FLAG) $(CXXDEFS) $(CXX_INCLUDES) $(CWARN) $(CEXTRA) $(CXXEXTRA) $(CYGEXTRA) $(CTUNING)

ASFLAGS += -Wa,-adhlns=$(<:.S=.lst),-gstabs $(CDEFS) $(C_INCLUDES)
LDFLAGS += $(LD_LIBS)

ALL_CFLAGS = $(CFLAGS)
ALL_CXXFLAGS = $(CXXFLAGS)
ALL_ASFLAGS = $(ASFLAGS)

ifeq ($(BUILD_HOST_PLATFORM),CYGWIN_NT-6.1)
CYGWIN_CC_DEFS = CC=i686-w64-mingw32-gcc CXX=i686-w64-mingw32-g++ CYGEXTRA=-DWIN32
endif
