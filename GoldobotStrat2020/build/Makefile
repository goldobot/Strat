#
# GoldobotStrat2020
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
MODULE_NAME := GoldobotStrat2020

include $(MAKE_TREE)/mak_def.inc

ifeq ($(MODULE_NAME),GoldobotStrat2020_sim)
CXXSRC += main.cpp goldo_conf.cpp comm_zmq.cpp sim/comm_rplidar_sim.cpp sim/comm_nucleo_sim.cpp sim/robot_state_sim.cpp sim/world_state_sim.cpp sim/virtual_robot.cpp detect/lidar_detect.cpp strat/robot_strat_base.cpp strat/robot_strat.cpp astar/astar.cpp
ifeq ($(EXTRA_FLAGS),-DROS)
EXTRA_CXXSRC := src/robot_simulator.cpp libs/goldo_robot/src/geometry/trajectory.cpp libs/goldo_robot/src/control/speed_controller.cpp libs/goldo_robot/src/control/trapezoidal_speed_profile.cpp libs/goldo_robot/src/control/propulsion_controller.cpp libs/goldo_robot/src/control/pid_controller.cpp libs/goldo_robot/src/control/propulsion_low_level_controller.cpp libs/goldo_robot/src/odometry/simple_odometry.cpp
EXTRA_CXXINC := -I$(EXTERNAL_PRJ_PATH)/libs/goldo_robot/include -I$(EXTERNAL_PRJ_PATH)/src
else 
EXTRA_CXXSRC :=
EXTRA_CXXINC :=
endif
else 
CXXSRC += main.cpp goldo_conf.cpp comm_zmq.cpp comm_rplidar.cpp comm_nucleo.cpp robot_state.cpp world_state.cpp detect/lidar_detect.cpp strat/robot_strat_base.cpp strat/robot_strat.cpp astar/astar.cpp
EXTRA_CXXSRC :=
EXTRA_CXXINC :=
endif

LD_LIBS += -lstdc++ -lpthread -lm -lzmq 
ifneq ($(YAML),import)
LD_LIBS += -lyaml-cpp
endif

all: build_app

sim:
	@make MODULE_NAME=GoldobotStrat2020_sim $(CYGWIN_CC_DEFS)

sim_ros:
	@if [ -z $(EXTERNAL_PRJ_PATH) ]; then \
		echo "You must define EXTERNAL_PRJ_PATH"; \
	else \
		make MODULE_NAME=GoldobotStrat2020_sim $(CYGWIN_CC_DEFS) EXTRA_FLAGS=-DROS;\
	fi

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
