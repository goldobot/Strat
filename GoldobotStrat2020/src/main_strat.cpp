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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <pthread.h>

#include <signal.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "direct_uart_nucleo.hpp"
#include "odometry_state.hpp"
#include "robot_detect.hpp"

using namespace goldobot;


char       * conf_viewer_addr_str      = "192.168.0.241";
double       conf_theta_correction_deg = 30.0f; /* PR 28/05/2019 */
char       * conf_rplidar_dev_str      = "/dev/rplidar";
_u32         conf_rplidar_baudrate     = 115200;
char       * conf_nucleo_uart_dev_str  = "/dev/odometry";
_u32         conf_nucleo_uart_baudrate = 115200;
_u32         conf_zmq_port             = 3101;


bool ctrl_c_pressed = false;
void ctrlc(int);

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
  /* FIXME : TODO */
  printf("GoldobotStrat2020 (%s)\n",my_firmware_ver);


//// Process command line parameters and read conf /////////////////////////////
  process_command_line(argc, argv);


#if 0 /* FIXME : DEBUG */
//// Initialise software components ////////////////////////////////////////////
  if (OdometryState::instance().init()!=0)
  {
    fprintf(stderr, "Error, cannot init odometry state.\n");
    return -1;
  }

  if (DirectUartNucleo::instance().init(conf_nucleo_uart_dev_str, conf_nucleo_uart_baudrate)!=0)
  {
    fprintf(stderr, "Error, cannot init nucleo uart direct interface.\n");
    return -1;
  }

  if (CommZmq::instance().init(conf_zmq_port)!=0)
  {
    fprintf(stderr, "Error, cannot init ZMQ interface.\n");
    return -1;
  }

  if (CommRplidar::instance().init(conf_rplidar_dev_str, conf_theta_correction_deg*M_PI/180.0f, conf_rplidar_baudrate)!=0)
  {
    fprintf(stderr, "Error, cannot init the rplidar interface.\n");
    return -1;
  }

  if (CommRplidar::instance().init_viewer_sock(conf_viewer_addr_str)!=0)
  {
    fprintf(stderr, "Error, cannot init the rplidar debug socket.\n");
    //return -1;
  }

  if (RobotDetect::instance().init()!=0)
  {
    fprintf(stderr, "Error, cannot init adversary tracker.\n");
    return -1;
  }
#endif


//// Install signal handler to detect CTRL_C ///////////////////////////////////
  ctrl_c_pressed = false;

  signal(SIGINT, ctrlc);


#if 0 /* FIXME : DEBUG */
//// Create and launch worker threads //////////////////////////////////////////
  if (OdometryState::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the odometry thread.\n");
    return -1;
  }

  if (DirectUartNucleo::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the nucleo uart thread.\n");
    return -1;
  }

  if (CommZmq::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the ZMQ thread.\n");
    return -1;
  }

  if (CommRplidar::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the rplidar thread.\n");
    return -1;
  }

#if 0 /* FIXME : DEBUG */
  if (RobotDetect::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the adversary tracker thread.\n");
    return -1;
  }
#endif
#endif


//// Main loop /////////////////////////////////////////////////////////////////
  while (1) {
    if (ctrl_c_pressed) 
    { 
      /* FIXME : TODO : stop threads */
      CommZmq::instance().stopTask();

      printf ("Bye!\n");
      break;
    }

    usleep (1000);

    pthread_yield();
  }

  /* FIXME : TODO : wait for all threads to terminate .. */

  return 0;
}


void ctrlc(int)
{
  ctrl_c_pressed = true;
}

int process_command_line(int argc, const char * argv[]) 
{
  char dummy_str[256];

  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read viewer address
  if (argc>1) strncpy(dummy_str, argv[1], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_viewer_addr_str = dummy_str;
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_viewer_addr_str      = %s\n", 
             conf_viewer_addr_str);

  // read theta correction
  if (argc>2) strncpy(dummy_str, argv[2], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_theta_correction_deg = strtod(dummy_str, NULL);
  } 
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_theta_correction_deg = %f\n", 
             conf_theta_correction_deg);

  // read rplidar device from the command line...
  if (argc>3) strncpy(dummy_str, argv[3], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_rplidar_dev_str = dummy_str;
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_rplidar_dev_str      = %s\n", 
             conf_rplidar_dev_str);

  // read rplidar baud rate from the command line...
  if (argc>4) strncpy(dummy_str, argv[4], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_rplidar_baudrate = strtoul(dummy_str, NULL, 10);
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_rplidar_baudrate     = %d\n", 
             conf_rplidar_baudrate);

  // read nucleo uart device from the command line...
  if (argc>5) strncpy(dummy_str, argv[5], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_nucleo_uart_dev_str = dummy_str;
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_nucleo_uart_dev_str  = %s\n", 
             conf_nucleo_uart_dev_str);

  // read nucleo uart baudrate from the command line...
  if (argc>6) strncpy(dummy_str, argv[6], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_nucleo_uart_baudrate = strtoul(dummy_str, NULL, 10);
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_nucleo_uart_baudrate = %d\n", 
             conf_nucleo_uart_baudrate);

  // read zmq port from the command line...
  if (argc>7) strncpy(dummy_str, argv[7], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_zmq_port = strtoul(dummy_str, NULL, 10);
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 
  printf ("  conf_zmq_port             = %d\n", 
             conf_zmq_port);

  return 0;
}

