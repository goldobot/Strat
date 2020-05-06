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

#include <fstream>
#include <iostream>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "lidar_detect.hpp"
#include "robot_strat.hpp"

#include "astar/astar.h"

using namespace goldobot;


char         conf_viewer_addr_str_def[]      = "192.168.0.241";
double       conf_theta_correction_deg_def   = 30.0f; /* PR 28/05/2019 */
char         conf_rplidar_dev_str_def[]      = "/dev/rplidar";
_u32         conf_rplidar_baudrate_def       = 115200;
char         conf_nucleo_uart_dev_str_def[]  = "/dev/odometry";
_u32         conf_nucleo_uart_baudrate_def   = 115200;
_u32         conf_zmq_port_def               = 3101;
char         conf_strat_file_str_def[]       = "strat.yaml";

char         conf_viewer_addr_str[128]       = {0};
double       conf_theta_correction_deg       = 0.0f;
char         conf_rplidar_dev_str[128]       = {0};
_u32         conf_rplidar_baudrate           = 0;
char         conf_nucleo_uart_dev_str[128]   = {0};
_u32         conf_nucleo_uart_baudrate       = 0;
_u32         conf_zmq_port                   = 0;
char         conf_strat_file_str[128]        = {0};


bool ctrl_c_pressed = false;
void ctrlc(int);

bool autotest_flag = false;
bool test_astar_flag = false;
bool test_strat_conf_flag = false;


void set_default_conf();
int parse_yaml_conf(const char * yaml_fname);
int process_command_line(int argc, const char * argv[]);
void display_conf();


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


//// Process command line parameters and read conf /////////////////////////////
  set_default_conf();

  if (parse_yaml_conf("conf/GoldobotStrat2020.yaml")!=0)
  {
    fprintf(stderr, "Error, cannot parse conf file.\n");
    return -1;
  }

  if (process_command_line(argc, argv)!=0)
  {
    fprintf(stderr, "Error, wrong command line arguments.\n");
    return -1;
  }

  display_conf();


//// Initialise software components ////////////////////////////////////////////
  printf(" Initialising software components .. \n");

  if (RobotState::instance().init()!=0)
  {
    fprintf(stderr, "Error, cannot init odometry state.\n");
    return -1;
  }

  if ((!autotest_flag) && (DirectUartNucleo::instance().init(conf_nucleo_uart_dev_str, conf_nucleo_uart_baudrate)!=0))
  {
    fprintf(stderr, "Error, cannot init nucleo uart direct interface.\n");
    return -1;
  }

  if ((!autotest_flag) && (CommZmq::instance().init(conf_zmq_port)!=0))
  {
    fprintf(stderr, "Error, cannot init ZMQ interface.\n");
    return -1;
  }

  if ((!autotest_flag) && (CommRplidar::instance().init(conf_rplidar_dev_str, conf_theta_correction_deg*M_PI/180.0f, conf_rplidar_baudrate)!=0))
  {
    fprintf(stderr, "Error, cannot init the rplidar interface.\n");
    return -1;
  }

  if ((!autotest_flag) && (CommRplidar::instance().init_viewer_sock(conf_viewer_addr_str)!=0))
  {
    fprintf(stderr, "Error, cannot init the rplidar debug socket.\n");
    //return -1;
  }

  if (LidarDetect::instance().init()!=0)
  {
    fprintf(stderr, "Error, cannot init adversary tracker.\n");
    return -1;
  }

  if (RobotStrat::instance().init(conf_strat_file_str)!=0)
  {
    fprintf(stderr, "Error, cannot init strategy module.\n");
    return -1;
  }
  printf(" Initialising software components DONE\n");


//// Install signal handler to detect CTRL_C ///////////////////////////////////
  ctrl_c_pressed = false;

  signal(SIGINT, ctrlc);


  if (autotest_flag)
  {
    if (test_astar_flag)
    {
      char dbg_fname[128];
      printf(" ASTAR TEST\n");
      strncpy(dbg_fname,"astar_test1.ppm",sizeof(dbg_fname));
      RobotStrat::instance().dbg_astar_test(1740,   290,
                                            1850, -1350,
                                             400, -1000,
                                            1600,  -300,
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
  if (LidarDetect::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the adversary tracker thread.\n");
    return -1;
  }
#endif

  if (RobotStrat::instance().startProcessing()!=0)
  {
    fprintf(stderr, "Error, cannot start the strategy thread.\n");
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
      DirectUartNucleo::instance().stopTask();
      CommRplidar::instance().stopTask();
      RobotStrat::instance().stopTask();

      break;
    }

    usleep (1000);

    pthread_yield();
  }

  /* wait for all threads to terminate .. */
  printf(" Waiting for all threads to terminate .. \n");
  for (int i=0; i<1000; i++) {
    if (!(
          CommZmq::instance().taskRunning() ||
          RobotState::instance().taskRunning() ||
          DirectUartNucleo::instance().taskRunning() ||
          CommRplidar::instance().taskRunning() ||
          RobotStrat::instance().taskRunning()
          )) 
    { 
      printf ("Bye!\n");
      break;
    }

    usleep (1000);

    pthread_yield();
  }


  return 0;
}


void ctrlc(int)
{
  ctrl_c_pressed = true;
}


void set_default_conf() 
{
  strncpy(conf_viewer_addr_str, conf_viewer_addr_str_def, 
          sizeof (conf_viewer_addr_str));
  conf_theta_correction_deg = conf_theta_correction_deg_def;
  strncpy(conf_rplidar_dev_str, conf_rplidar_dev_str_def, 
          sizeof (conf_rplidar_dev_str));
  conf_rplidar_baudrate = conf_rplidar_baudrate_def;
  strncpy(conf_nucleo_uart_dev_str, conf_nucleo_uart_dev_str_def, 
          sizeof (conf_nucleo_uart_dev_str));
  conf_nucleo_uart_baudrate = conf_nucleo_uart_baudrate_def;
  conf_zmq_port = conf_zmq_port_def;
  strncpy(conf_strat_file_str, conf_strat_file_str_def, 
          sizeof (conf_strat_file_str));
}

int process_command_line(int argc, const char * argv[]) 
{
  char dummy_str[256];

  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

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
  }

  // read viewer address
  if (argc>1) strncpy(dummy_str, argv[1], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    strncpy(conf_viewer_addr_str, dummy_str, 
            sizeof (conf_viewer_addr_str));
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read theta correction
  if (argc>2) strncpy(dummy_str, argv[2], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_theta_correction_deg = strtod(dummy_str, NULL);
  } 
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read rplidar device from the command line...
  if (argc>3) strncpy(dummy_str, argv[3], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    strncpy(conf_rplidar_dev_str, dummy_str, 
            sizeof (conf_rplidar_dev_str));
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read rplidar baud rate from the command line...
  if (argc>4) strncpy(dummy_str, argv[4], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_rplidar_baudrate = strtoul(dummy_str, NULL, 10);
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read nucleo uart device from the command line...
  if (argc>5) strncpy(dummy_str, argv[5], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    strncpy(conf_nucleo_uart_dev_str, dummy_str, 
            sizeof (conf_nucleo_uart_dev_str));
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read nucleo uart baudrate from the command line...
  if (argc>6) strncpy(dummy_str, argv[6], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_nucleo_uart_baudrate = strtoul(dummy_str, NULL, 10);
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read zmq port from the command line...
  if (argc>7) strncpy(dummy_str, argv[7], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    conf_zmq_port = strtoul(dummy_str, NULL, 10);
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  // read strategy file name from the command line...
  if (argc>8) strncpy(dummy_str, argv[8], sizeof (dummy_str));
  if ((dummy_str[0]!='!') && (dummy_str[0]!='*')) 
  {
    strncpy(conf_strat_file_str, dummy_str, 
            sizeof (conf_strat_file_str));
  }
  memset (dummy_str, 0, sizeof (dummy_str));
  dummy_str[0] = '!'; dummy_str[1] = 0x00; 

  return 0;
}

void display_conf() 
{
  printf ("  conf_viewer_addr_str      = %s\n", 
             conf_viewer_addr_str);
  printf ("  conf_theta_correction_deg = %f\n", 
             conf_theta_correction_deg);
  printf ("  conf_rplidar_dev_str      = %s\n", 
             conf_rplidar_dev_str);
  printf ("  conf_rplidar_baudrate     = %d\n", 
             conf_rplidar_baudrate);
  printf ("  conf_nucleo_uart_dev_str  = %s\n", 
             conf_nucleo_uart_dev_str);
  printf ("  conf_nucleo_uart_baudrate = %d\n", 
             conf_nucleo_uart_baudrate);
  printf ("  conf_zmq_port             = %d\n", 
             conf_zmq_port);
  printf ("  conf_strat_file_str       = %s\n", 
             conf_strat_file_str);
}


int parse_yaml_conf(const char * yaml_fname)
{
  int ret = 0;
  std::ifstream fin;
  const char *my_str = NULL;

  try 
  {
    fin.open(yaml_fname);

    YAML::Node yconf = YAML::Load(fin);
    YAML::Node test_node;

    test_node = yconf["environment"]["conf_viewer_addr_str"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      strncpy(conf_viewer_addr_str, my_str, 
              sizeof (conf_viewer_addr_str));
    }

    test_node = yconf["environment"]["conf_theta_correction_deg"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      conf_theta_correction_deg = strtod(my_str, NULL);
    }

    test_node = yconf["environment"]["conf_rplidar_dev_str"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      strncpy(conf_rplidar_dev_str, my_str, 
              sizeof (conf_rplidar_dev_str));
    }

    test_node = yconf["environment"]["conf_rplidar_baudrate"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      conf_rplidar_baudrate = strtoul(my_str, NULL, 10);
    }

    test_node = yconf["environment"]["conf_nucleo_uart_dev_str"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      strncpy(conf_nucleo_uart_dev_str, my_str, 
              sizeof (conf_nucleo_uart_dev_str));
    }

    test_node = yconf["environment"]["conf_nucleo_uart_baudrate"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      conf_nucleo_uart_baudrate = strtoul(my_str, NULL, 10);
    }

    test_node = yconf["environment"]["conf_zmq_port"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      conf_zmq_port = strtoul(my_str, NULL, 10);
    }

    test_node = yconf["environment"]["conf_strat_file_str"];
    if (test_node) 
    {
      my_str = (const char *) test_node.as<std::string>().c_str();
      strncpy(conf_strat_file_str, my_str, 
              sizeof (conf_strat_file_str));
    }

    ret = 0;
  } 
  catch(const YAML::Exception& e)
  {
    std::cerr << e.what() << "\n";
    ret = -1;
  }

  return ret;
}
