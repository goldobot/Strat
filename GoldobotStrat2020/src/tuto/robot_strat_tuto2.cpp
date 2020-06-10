#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#include <string.h>
#include <unistd.h>
#include <math.h>
#include <zmq.h>

#include <fstream>
#include <iostream>

#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "world_state.hpp"
#include "detect/lidar_detect.hpp"
#include "strat/robot_strat_types.hpp"
//#include "strat/robot_strat_base.hpp"

#include "tuto/robot_strat_tuto2.hpp"


using namespace goldobot;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define MAX(a,b) ((a>b)?a:b)


/******************************************************************************/
/**  RobotStrat  **************************************************************/
/******************************************************************************/

RobotStrat RobotStrat::s_instance;

RobotStrat& RobotStrat::instance()
{
  return s_instance;
}

RobotStrat::RobotStrat()
{
  strncpy(m_thread_name,"RobotStrat",sizeof(m_thread_name));

  memset (m_strat_file_name, 0, sizeof(m_strat_file_name));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  m_start_match_sig = false;

  /* FIXME : TODO : other inits in constructor */

}

int RobotStrat::init(char *strat_file_name)
{
  strncpy(m_strat_file_name, strat_file_name, sizeof(m_strat_file_name));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  m_start_match_sig = false;

  /* FIXME : TODO : other inits in constructor */

  read_yaml_conf (m_strat_file_name);

  return 0;
}

int RobotStrat::read_yaml_conf(char *fname)
{
  std::ifstream fin;
  YAML::Node yconf;
  YAML::Node dbg_node;

  try 
  {
    fin.open(fname);
    yconf = YAML::Load(fin);
  } 
  catch(const YAML::Exception& e)
  {
    printf ("  ERROR : %s\n", e.what());
    return -1;
  }

  /* FIXME : TODO */

  return 0;
}

void RobotStrat::taskFunction()
{
  struct timespec my_tp;
  unsigned int my_time_ms = 0;
  unsigned int last_log_time_ms = 0;
  unsigned int log_period_ms = 10000; /* 10 sec */

  m_task_running = true;

  /* FIXME : TODO : synchronise with comm threads to avoid fake event 
     detection */
#ifndef WIN32
  usleep(100000);
#else
  Sleep(100);
#endif

  /*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  /*++ TUTO2 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  /*++ Recuperation de la position du robot et des adversaires detectes +++++*/
  /*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

  printf ("\n\n");

  int x_start_mm = RobotState::instance().s().x_mm;
  int y_start_mm = RobotState::instance().s().y_mm;
  printf ("TUTO2 : robot initial position (mm): <%d,%d>\n", 
          x_start_mm, y_start_mm);

  printf ("\n\n");

  while(!m_stop_task)
  {

    clock_gettime(1, &my_tp);
    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
    my_time_ms = my_time_ms; /* stupid compilator!.. */

    if (my_time_ms > (last_log_time_ms + log_period_ms))
    {
      printf ("\n");

      /* FIXME : TODO */
      int my_x_mm = RobotState::instance().s().x_mm;
      int my_y_mm = RobotState::instance().s().y_mm;
      printf ("TUTO2 : my_pose (mm): <%d,%d>\n", my_x_mm, my_y_mm);

      WorldState::instance().lock();
      detected_robot_info_t& o0 = 
        WorldState::instance().detected_robot(0);
      detected_robot_info_t& o1 = 
        WorldState::instance().detected_robot(1);
      detected_robot_info_t& o2 = 
        WorldState::instance().detected_robot(2);
      WorldState::instance().release();

      printf ("TUTO2 : robot0 (mm): <%d,%d>\n", (int) o0.x_mm, (int) o0.y_mm);
      printf ("TUTO2 : robot1 (mm): <%d,%d>\n", (int) o1.x_mm, (int) o1.y_mm);
      printf ("TUTO2 : robot2 (mm): <%d,%d>\n", (int) o2.x_mm, (int) o2.y_mm);

      last_log_time_ms = my_time_ms;
    }

#ifndef WIN32
    usleep(10000);
#else
    Sleep(10);
#endif

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  } /* while(!m_stop_task) */

  /*-------------------------------------------------------------------------*/
  /*-- TUTO2 ----------------------------------------------------------------*/
  /*-------------------------------------------------------------------------*/

  m_task_running = false;
}

void RobotStrat::start_match()
{
  if (!m_task_running) return;

  m_start_match_sig = true;
}

void RobotStrat::dbg_pause_match()
{
  if (!m_task_running) return;

  /* FIXME : DEBUG */
}

void RobotStrat::dbg_resume_match()
{
  if (!m_task_running) return;

  /* FIXME : DEBUG */
}

int RobotStrat::cmd_traj(strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0055; /*DbgPropulsionExecuteTrajectory*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&speed, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&accel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&deccel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  for (int i=0; i<_nwp; i++) 
  {
    float my_x = _wp[i].x_mm*0.001;
    float my_y = _wp[i].y_mm*0.001;

    field_len = sizeof(float);
    memcpy (_pc, (unsigned char *)&(my_x), field_len);
    _pc += field_len;
    cmd_buf_len += field_len;

    field_len = sizeof(float);
    memcpy (_pc, (unsigned char *)&(my_y), field_len);
    _pc += field_len;
    cmd_buf_len += field_len;
  }

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_point_to(strat_way_point_t *_wp, float speed, float accel, float deccel)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0058; /*DbgPropulsionExecutePointTo*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = _wp->x_mm*0.001;
  float my_y = _wp->y_mm*0.001;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&(my_x), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&(my_y), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&speed, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&accel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&deccel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_set_pose(float x_mm, float y_mm, float theta_deg)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0053; /*DbgPropulsionSetPose*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = x_mm*0.001;
  float my_y = y_mm*0.001;
  float my_theta = theta_deg*M_PI/180.0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&(my_x), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&(my_y), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&my_theta, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_nucleo_seq (unsigned int seq_id)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x002b; /*MainSequenceStartSequence*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;
  unsigned short seq_id_s = (unsigned short) seq_id;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&(seq_id_s), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_clear_prop_err()
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0063; /*PropulsionClearError*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

void RobotStrat::dbg_astar_test(int x_start_mm, int y_start_mm,
                                int x_end_mm, int y_end_mm,
                                int xo1_mm, int yo1_mm,
                                int xo2_mm, int yo2_mm,
                                int xo3_mm, int yo3_mm,
                                char *dump_fname)
{
  x_start_mm = x_start_mm;
  y_start_mm = y_start_mm;
  x_end_mm = x_end_mm;
  y_end_mm = y_end_mm;
  xo1_mm = xo1_mm;
  yo1_mm = yo1_mm;
  xo2_mm = xo2_mm;
  yo2_mm = yo2_mm;
  xo3_mm = xo3_mm;
  yo3_mm = yo3_mm;
  printf ("ERROR : RobotStrat::dbg_astar_test() was disabled\n");
}

void RobotStrat::dbg_dump()
{
  printf ("ERROR : RobotStrat::dbg_dump() was disabled\n");
}





