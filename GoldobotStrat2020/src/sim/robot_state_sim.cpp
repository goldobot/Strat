#ifdef WIN32
#include <windows.h>
#endif

#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#define ROBOT_SIM 1

#define SIM_GRANULARITY 10000

#include "robot_state.hpp"


using namespace goldobot;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

RobotState RobotState::s_instance;

RobotState& RobotState::instance()
{
  return s_instance;
}

RobotState::RobotState()
{
  strncpy(m_thread_name,"RobotState_sim",sizeof(m_thread_name));

  m_stop_task = false;
  m_task_running = false;

  m_s.local_ts_ms  = 0;
  m_s.remote_ts_ms = 0;
  m_s.x_mm         = 0;
  m_s.y_mm         = 0;
  m_s.theta_deg    = 0.0;

  m_s.speed_abs = 0.0;
  m_s.forward_move = true;

  m_s.robot_sensors = 0;

  pthread_mutex_init(&m_lock, NULL);
}

int RobotState::init()
{
  m_s.local_ts_ms  = 0;
  m_s.remote_ts_ms = 0;
  m_s.x_mm         = 0;
  m_s.y_mm         = 0;
  m_s.theta_deg    = 0.0;

  m_s.speed_abs = 0.0;
  m_s.forward_move = true;

  m_s.robot_sensors = GPIO_START_MASK;

  pthread_mutex_init(&m_lock, NULL);

  return 0;
}

#define MAX(a,b) ((a>b)?a:b)

void RobotState::taskFunction()
{
  unsigned int time_ms = 0;
  unsigned int time_ms_old = 0;
  int delta_time_ms = 0;
  struct timespec my_tp;
  volatile int odo_x_mm = 0;
  volatile int odo_y_mm = 0;
  volatile double odo_theta_deg = 0.0;
  volatile double odo_theta_rad = 0.0;
  int odo_x_mm_old = 0;
  int odo_y_mm_old = 0;
  int delta_x_mm = 0;
  int delta_y_mm = 0;
  double delta_d_mm = 0.0;

  int dbg_cnt = 0;

  m_s.speed_abs = 0.0;
  m_s.forward_move = true;
  m_s.speed_abs = 0.0;

  m_task_running = true;

  while(!m_stop_task)
  {
    /**  Time  ****************************************************************/

    clock_gettime(1, &my_tp);

    time_ms_old = time_ms;
    time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
    m_s.local_ts_ms = time_ms;
    delta_time_ms = time_ms - time_ms_old;

    /**  Backup of the cinematic state and computing of the differentials  ****/

    odo_x_mm_old = odo_x_mm;
    odo_y_mm_old = odo_y_mm;

    lock();
    odo_x_mm      = m_s.x_mm;
    odo_y_mm      = m_s.y_mm;
    odo_theta_deg = m_s.theta_deg;
    release();

    delta_x_mm = odo_x_mm - odo_x_mm_old;
    delta_y_mm = odo_y_mm - odo_y_mm_old;
    delta_d_mm = sqrt(delta_x_mm*delta_x_mm + delta_y_mm*delta_y_mm);

    odo_theta_rad = odo_theta_deg*M_PI/180.0;

    /**  Detection of the sense of moving  ************************************/

    if (delta_time_ms!=0) 
    {
      // result in m/sec
      m_s.speed_abs = delta_d_mm/delta_time_ms;
      if (m_s.speed_abs>0.005) 
      {
        m_s.forward_move = ((delta_x_mm*cos(odo_theta_rad) + 
                             delta_y_mm*sin(odo_theta_rad)) > 0.0);
      }
    }

    /**  Iteration end in the main loop of the simulation  thread  ************/

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

    dbg_cnt++;
  } /* while(!m_stop_task) */

  m_task_running = false;
}

void RobotState::get_s(robot_state_info_t *_s)
{
  lock();
  *_s = m_s;
  release();
}

void RobotState::set_s(robot_state_info_t *_s)
{
  lock();
  m_s = *_s;
  release();
}

int RobotState::lock(int timeout_ms)
{
  if (timeout_ms < 0)
  {
    if (pthread_mutex_lock(&m_lock) == 0)
    {
      return 0;
    }
  }
  else if (timeout_ms == 0)
  {
    if (pthread_mutex_trylock(&m_lock) == 0)
    {
      return 0;
    }
  }
  else
  {
    timespec wait_time;
    timeval now;
    gettimeofday(&now,NULL);

    wait_time.tv_sec = timeout_ms/1000 + now.tv_sec;
    wait_time.tv_nsec = (timeout_ms%1000)*1000000 + now.tv_usec*1000;
        
    if (pthread_mutex_timedlock(&m_lock,&wait_time) == 0)
    {
      return 0;
    }
  }

  return -1;
}

void RobotState::release()
{
  pthread_mutex_unlock(&m_lock);
}

