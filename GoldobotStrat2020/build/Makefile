#/*
# * Copyright (C) 2014  RoboPeak
# *
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
MODULE_NAME := GoldobotStrat2020

include $(MAKE_TREE)/mak_def.inc

CXXSRC += main_strat.cpp comm_zmq.cpp comm_rplidar.cpp direct_uart_nucleo.cpp odometry_state.cpp robot_detect.cpp

EXTRA_OBJ := 
LD_LIBS += -lstdc++ -lpthread -lm -lzmq 

all: build_app

include $(MAKE_TREE)/mak_common.inc

clean: clean_app

print_bidule:
	@echo "BUILD_TARGET_ARCH = $(BUILD_TARGET_ARCH)"
	@echo "BUILD_HOST_PLATFORM = $(BUILD_HOST_PLATFORM)"
	@echo "BUILD_HOST_HW_PLATFORM = $(BUILD_HOST_HW_PLATFORM)"
	@echo "RPLIDAR_SDK_IMPORT = $(RPLIDAR_SDK_IMPORT)"
