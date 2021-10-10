/*
 *  GoldobotStrat2020
 *
 *  Copyright (c) 2020 Goldorak team
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifdef WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include <unistd.h>

#include <pthread.h>

#include <signal.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "goldo_conf.hpp"
#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "world_state.hpp"
#include "detect/lidar_detect.hpp"
#include "strat/robot_strat.hpp"

#include "astar/astar.h"

using namespace goldobot;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

bool ctrl_c_pressed = false;
void ctrlc(int);

int debug_flags = 0;
bool autotest_flag = false;
bool test_astar_flag = false;
bool test_strat_conf_flag = false;


int process_command_line(int argc, const char * argv[]);


#ifdef GOLDO_GIT_VERSION
#define _STRINGIFY(A) #A
#define STRINGIFY(A) _STRINGIFY(A)
#define MY_GIT_VERSION STRINGIFY(GOLDO_GIT_VERSION)
#else
#define MY_GIT_VERSION "GOLDO HACK"
#endif

#define MY_FIRMWARE_VER_SZ 64
const char my_firmware_ver[MY_FIRMWARE_VER_SZ] = MY_GIT_VERSION;


int main(int argc, const char * argv[]) 
{

//// Print version /////////////////////////////////////////////////////////////
  printf("GoldobotStrat2020 (%s)\n",my_firmware_ver);


//// Read conf and process command line parameters /////////////////////////////
  if (GoldoConf::instance().init("conf/GoldobotStrat2020.yaml")!=0)
  {
    fprintf(stderr, "ERROR : cannot parse conf file.\n");
    return -1;
  }

  if (process_command_line(argc, argv)!=0)
  {
    fprintf(stderr, "ERROR : wrong command line arguments.\n");
    return -1;
  }

  GoldoConf::instance().display_conf();


//// Initialise software components ////////////////////////////////////////////
  printf(" Initialising software components .. \n");
  goldo_conf_info_t& ci = GoldoConf::instance().c();

  if (RobotState::instance().init()!=0)
  {
    fprintf(stderr, "ERROR : cannot init odometry state.\n");
    return -1;
  }

  if (WorldState::instance().init()!=0)
  {
    fprintf(stderr, "ERROR : cannot init 'world representation' state.\n");
    return -1;
  }

  if ((!autotest_flag) && (DirectUartNucleo::instance().init(ci.conf_nucleo_uart_dev_str, ci.conf_nucleo_uart_baudrate)!=0))
  {
    fprintf(stderr, "ERROR : cannot init nucleo uart direct interface.\n");
    return -1;
  }

  if ((!autotest_flag) && (CommZmq::instance().init(ci.conf_zmq_port)!=0))
  {
    fprintf(stderr, "ERROR : cannot init ZMQ interface.\n");
    return -1;
  }

  if ((!autotest_flag) && (CommRplidar::instance().init(ci.conf_rplidar_dev_str, ci.conf_rplidar_baudrate)!=0))
  {
    fprintf(stderr, "ERROR : cannot init the rplidar interface.\n");
    return -1;
  }

  if ((!autotest_flag) && (CommRplidar::instance().init_viewer_sock(ci.conf_viewer_addr_str)!=0))
  {
    fprintf(stderr, "ERROR : cannot init the rplidar debug socket.\n");
    //return -1;
  }

  if (LidarDetect::instance().init()!=0)
  {
    fprintf(stderr, "ERROR : cannot init adversary tracker.\n");
    return -1;
  }
#if 1 /* FIXME : DEBUG */
  LidarDetect::instance().set_nb_of_send_detect(ci.conf_n_obstacles);
#endif

  if (RobotStrat::instance().init(ci.conf_strat_file_str)!=0)
  {
    fprintf(stderr, "ERROR : cannot init strategy module.\n");
    return -1;
  }
  printf(" Initialising software components DONE\n");


//// Install signal handler to detect CTRL_C ///////////////////////////////////
  ctrl_c_pressed = false;

  signal(SIGINT, ctrlc);


  if (debug_flags!=0)
  {
    printf ("DEBUG : debug_flags=%d\n",debug_flags);
    RobotStrat::instance().set_debug(debug_flags);
  }

  if (autotest_flag)
  {
    if (test_astar_flag)
    {
      char dbg_fname[128];
      printf(" ASTAR TEST\n");
      strncpy(dbg_fname,"astar_test1.ppm",sizeof(dbg_fname));
      RobotStrat::instance().dbg_astar_test(1650,   290,
                                            1850, -1350,
                                             400, -1000,
                                             800,  -300,
                                             200,   650,
                                            dbg_fname);
    }
    else if (test_strat_conf_flag)
    {
      printf(" STRAT CONF TEST\n");
      RobotStrat::instance().dbg_dump();
    }
    return 0;
  }


//// Create and launch worker threads //////////////////////////////////////////
  printf(" Creating and launching worker threads .. \n");

  if (RobotState::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the odometry thread.\n");
    return -1;
  }

  if (WorldState::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the 'world representation' thread.\n");
    return -1;
  }

  if (DirectUartNucleo::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the nucleo uart thread.\n");
    return -1;
  }

  if (CommZmq::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the ZMQ thread.\n");
    return -1;
  }

  if (CommRplidar::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the rplidar thread.\n");
    return -1;
  }

#if 0 /* FIXME : DEBUG */
  if (LidarDetect::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the adversary tracker thread.\n");
    return -1;
  }
#endif

  if (RobotStrat::instance().startProcessing()!=0)
  {
    fprintf(stderr, "ERROR : cannot start the strategy thread.\n");
    return -1;
  }
  printf(" Creating and launching worker threads DONE\n");


//// Main loop /////////////////////////////////////////////////////////////////
  while (1) {
    if (ctrl_c_pressed) 
    { 
      printf ("\nStopping worker threads..\n");

      CommZmq::instance().stopTask();
      RobotState::instance().stopTask();
      WorldState::instance().stopTask();
      DirectUartNucleo::instance().stopTask();
      CommRplidar::instance().stopTask();
      RobotStrat::instance().stopTask();

      break;
    }

#ifndef WIN32
    usleep(1000);
#else
    Sleep(1);
#endif

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  }

  /* wait for all threads to terminate .. */
  printf(" Waiting for all threads to terminate .. \n");
  for (int i=0; i<1000; i++) {
    if (!(
          CommZmq::instance().taskRunning() ||
          RobotState::instance().taskRunning() ||
          WorldState::instance().taskRunning() ||
          DirectUartNucleo::instance().taskRunning() ||
          CommRplidar::instance().taskRunning() ||
          RobotStrat::instance().taskRunning()
          )) 
    { 
      printf ("Bye!\n");
      break;
    }

#ifndef WIN32
    usleep(1000);
#else
    Sleep(1);
#endif

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  }


  return 0;
}


void ctrlc(int)
{
  ctrl_c_pressed = true;
}


int process_command_line(int argc, const char * argv[]) 
{

  // detect autotest request
  if (argc>1)
  {
    if (strncmp(argv[1],"test",4)==0)
    {
      autotest_flag = true;
      if (strncmp(argv[1],"test_astar",10)==0)
      {
        test_astar_flag = true;
      } 
      else if (strncmp(argv[1],"test_strat_conf",15)==0) 
      {
        test_strat_conf_flag = true;
      }
      return 0;
    }
    if (strncmp(argv[1],"debug",5)==0)
    {
      if (argc>2)
      {
        debug_flags = atoi(argv[2]);
      } 
      else
      {
        debug_flags = 3;
      }
      return 0;
    }
  }

  return 0;
}

