#include <string.h>
#include <unistd.h>
#include <math.h>

#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "lidar_detect.hpp"
#include "robot_strat.hpp"


using namespace goldobot;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


typedef struct _dbg_point_t {
  float x;
  float y;
} dbg_point_t;

dbg_point_t dbg_point[] = {
  { 0.556, -1.133},
  { 0.336, -0.990},
  { 0.233, -0.950},
  { 0.223, -0.726},
  { 0.626, -0.396},
  { 1.013, -0.470},
  { 1.280, -0.096},
  { 1.153,  0.336},
  { 1.300,  0.440},
  { 1.490,  0.300},
  { 1.740,  0.290},
};

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

  /* FIXME : TODO */
  m_dbg_start_match = false;
}

int RobotStrat::init(char *strat_file_name)
{
  strncpy(m_strat_file_name, strat_file_name, sizeof(m_strat_file_name));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  /* FIXME : TODO */
  m_dbg_start_match = false;

  return 0;
}

#define MAX(a,b) ((a>b)?a:b)

void RobotStrat::taskFunction()
{
  bool dbg_match_started = false;

  m_task_running = true;

  while(!m_stop_task)
  {

    /* FIXME : TODO */
    volatile unsigned int my_robot_sensors = 
      RobotState::instance().m_robot_sensors;

    if (((my_robot_sensors&2)==0) && (!dbg_match_started))
    {
      printf ("\n DEBUG : Tirette!..\n\n");

      usleep (3000000);

      m_dbg_start_match = true;
    }

    if (m_dbg_start_match && (!dbg_match_started))
    {
      dbg_match_started = true;

      int num_points = _countof(dbg_point);
      float speed = 0.4;
      float accel = 0.3;
      float deccel = 0.3;

      int cmd_buf_len = 0;
      unsigned char *_pc = m_nucleo_cmd_buf;
      unsigned short int *_ps = NULL;
      float *_pf = NULL;

      _ps = (unsigned short int *)_pc;
      *_ps = 0x0055;
      _pc += sizeof(unsigned short int);
      cmd_buf_len += sizeof(unsigned short int);

      _pf = (float *)_pc;
      *_pf = speed;
      _pc += sizeof(float);
      cmd_buf_len += sizeof(float);

      _pf = (float *)_pc;
      *_pf = accel;
      _pc += sizeof(float);
      cmd_buf_len += sizeof(float);

      _pf = (float *)_pc;
      *_pf = deccel;
      _pc += sizeof(float);
      cmd_buf_len += sizeof(float);

      for (int i=0; i<num_points; i++) 
      {
        _pf = (float *)_pc;
        *_pf = dbg_point[i].x;
        _pc += sizeof(float);
        cmd_buf_len += sizeof(float);

        _pf = (float *)_pc;
        *_pf = dbg_point[i].y;
        _pc += sizeof(float);
        cmd_buf_len += sizeof(float);
      }

      DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);
    }

    usleep (10000);

    pthread_yield();
  } /* while(!m_stop_task) */

  m_task_running = false;
}

void RobotStrat::start_match()
{
  if (!m_task_running) return;

  /* FIXME : TODO */
  m_dbg_start_match = true;

}

