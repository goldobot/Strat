#
# GoldobotDetect2020
#
# Copyright (c) 2020 Goldorak team
#
#
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *
# */
#
#HOME_TREE := .
HOME_TREE := ..
MAKE_TREE := .

#MODULE_NAME := $(notdir $(CURDIR))
MODULE_NAME := GoldobotDetect2020

include $(MAKE_TREE)/mak_def.inc

CXXSRC += main.cpp goldo_conf.cpp comm_zmq.cpp comm_rplidar.cpp comm_nucleo.cpp robot_state.cpp detect/lidar_detect.cpp
EXTRA_CXXSRC :=
EXTRA_CXXINC :=

LD_LIBS += -lstdc++ -lpthread -lm -lzmq 
ifneq ($(YAML),import)
LD_LIBS += -lyaml-cpp
endif

all: build_app


include $(MAKE_TREE)/mak_common.inc

clean: clean_app
	rm -f *.ppm

print_bidule:
	@echo "MODULE_NAME = $(MODULE_NAME)"
	@echo "BUILD_TARGET_ARCH = $(BUILD_TARGET_ARCH)"
	@echo "BUILD_HOST_PLATFORM = $(BUILD_HOST_PLATFORM)"
	@echo "BUILD_HOST_HW_PLATFORM = $(BUILD_HOST_HW_PLATFORM)"
	@echo "RPLIDAR_SDK_IMPORT = $(RPLIDAR_SDK_IMPORT)"
	@echo 'CXXSRC = $(CXXSRC)'
	@echo 'EXTRA_CXXSRC = $(EXTRA_CXXSRC)'
