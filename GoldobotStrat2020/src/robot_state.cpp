#ifdef WIN32
#include <windows.h>
#endif

#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#include "robot_state.hpp"


using namespace goldobot;


RobotState RobotState::s_instance;

RobotState& RobotState::instance()
{
  return s_instance;
}

RobotState::RobotState()
{
  strncpy(m_thread_name,"RobotState",sizeof(m_thread_name));

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
  unsigned int l_dbg_thread_time_ms_old;
  unsigned int l_dbg_thread_time_ms_delta;
  unsigned int l_dbg_thread_time_ms_delta_max;

  unsigned int l_uart_thread_time_ms_old;
  unsigned int l_odo_time_ms_old;
  int          l_odo_x_mm_old;
  int          l_odo_y_mm_old;
  double       l_odo_theta_deg_old;
  double       l_odo_theta_rad;

  unsigned int l_uart_thread_time_ms_delta;
  unsigned int l_odo_time_ms_delta;
  int          l_odo_x_mm_delta;
  int          l_odo_y_mm_delta;
  int          l_odo_d_mm_delta;
  double       l_odo_theta_deg_delta;

  unsigned int l_uart_thread_time_ms_delta_max;
  unsigned int l_odo_time_ms_delta_max;
  int          l_odo_d_mm_delta_max;
  double       l_odo_theta_deg_delta_max;

  int dbg_cnt = 0;


  m_s.speed_abs = 0.0;
  m_s.forward_move = true;

  l_dbg_thread_time_ms_old = 0;
  l_dbg_thread_time_ms_delta = 0;
  l_dbg_thread_time_ms_delta_max = 0;

  l_uart_thread_time_ms_old = 0;
  l_odo_time_ms_old = 0;
  l_odo_x_mm_old = 0;
  l_odo_y_mm_old = 0;
  l_odo_theta_deg_old = 0.0;
  l_odo_theta_rad = 0.0;

  l_uart_thread_time_ms_delta = 0;
  l_odo_time_ms_delta = 0;
  l_odo_x_mm_delta = 0;
  l_odo_y_mm_delta = 0;
  l_odo_d_mm_delta = 0;
  l_odo_theta_deg_delta = 0.0;

  l_uart_thread_time_ms_delta_max = 0;
  l_odo_time_ms_delta_max = 0;
  l_odo_d_mm_delta_max = 0;
  l_odo_theta_deg_delta_max = 0.0;


  m_task_running = true;

  while(!m_stop_task)
  {

    //if ((dbg_cnt%10)==0) 
    if (m_s.local_ts_ms!=l_uart_thread_time_ms_old) 
    {
      struct timespec my_tp;
      unsigned int dbg_thread_time_ms;

      lock();
      volatile unsigned int my_uart_thread_time_ms = m_s.local_ts_ms;
      volatile unsigned int my_odo_time_ms         = m_s.remote_ts_ms;
      volatile int          my_odo_x_mm            = m_s.x_mm;
      volatile int          my_odo_y_mm            = m_s.y_mm;
      volatile double       my_odo_theta_deg       = m_s.theta_deg;
      volatile unsigned int my_robot_sensors       = m_s.robot_sensors;
      release();

      clock_gettime(1, &my_tp);

      dbg_thread_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

      if (l_dbg_thread_time_ms_old != 0) 
      {
        l_dbg_thread_time_ms_delta = dbg_thread_time_ms - l_dbg_thread_time_ms_old;
        l_dbg_thread_time_ms_delta_max = MAX(l_dbg_thread_time_ms_delta_max, l_dbg_thread_time_ms_delta);
        l_dbg_thread_time_ms_old = dbg_thread_time_ms;

        l_uart_thread_time_ms_delta = abs(my_uart_thread_time_ms - l_uart_thread_time_ms_old);
        l_odo_time_ms_delta = abs(my_odo_time_ms - l_odo_time_ms_old);
        l_odo_x_mm_delta = my_odo_x_mm - l_odo_x_mm_old;
        l_odo_y_mm_delta = my_odo_y_mm - l_odo_y_mm_old;
        l_odo_d_mm_delta = sqrt(l_odo_x_mm_delta*l_odo_x_mm_delta + l_odo_y_mm_delta*l_odo_y_mm_delta);
        l_odo_theta_deg_delta = fabs(my_odo_theta_deg - l_odo_theta_deg_old);

        l_uart_thread_time_ms_delta_max = MAX(l_uart_thread_time_ms_delta_max, l_uart_thread_time_ms_delta);
        l_odo_time_ms_delta_max = MAX(l_odo_time_ms_delta_max, l_odo_time_ms_delta);
        l_odo_d_mm_delta_max = MAX(l_odo_d_mm_delta_max, l_odo_d_mm_delta);
        l_odo_theta_deg_delta_max = MAX(l_odo_theta_deg_delta_max, l_odo_theta_deg_delta);

        l_uart_thread_time_ms_old = my_uart_thread_time_ms;
        l_odo_time_ms_old = my_odo_time_ms;
        l_odo_x_mm_old = my_odo_x_mm;
        l_odo_y_mm_old = my_odo_y_mm;
        l_odo_theta_deg_old = my_odo_theta_deg;

        l_odo_theta_rad = my_odo_theta_deg*M_PI/180.0;

        if (l_odo_time_ms_delta==0) m_s.speed_abs = 0.0;
        else m_s.speed_abs = ((double)l_odo_d_mm_delta/*/1000.0*/)/(l_odo_time_ms_delta/*/1000.0*/); // result in m/sec
        if (m_s.speed_abs>0.005) 
        {
          m_s.forward_move = ((l_odo_x_mm_delta*cos(l_odo_theta_rad) + l_odo_y_mm_delta*sin(l_odo_theta_rad)) > 0.0);
        }
      } 
      else 
      {
        l_dbg_thread_time_ms_old = dbg_thread_time_ms;
        l_uart_thread_time_ms_old = my_uart_thread_time_ms;
        l_odo_time_ms_old = my_odo_time_ms;
        l_odo_x_mm_old = my_odo_x_mm;
        l_odo_y_mm_old = my_odo_y_mm;
        l_odo_theta_deg_old = my_odo_theta_deg;
      }

#if 0 /* FIXME : DEBUG : USEFULL (don't remove!) */
      printf ("dbg_thread_time_ms = %u (%u)\n", dbg_thread_time_ms, l_dbg_thread_time_ms_delta_max);
      printf ("my_uart_thread_time_ms = %u (%u)\n", my_uart_thread_time_ms, l_uart_thread_time_ms_delta_max);
      printf ("ts = %u (%u); pos = < %d , %d > (%u); theta = %f (%f)\n",
              my_odo_time_ms, l_odo_time_ms_delta_max, 
              my_odo_x_mm, my_odo_y_mm, l_odo_d_mm_delta_max,
              my_odo_theta_deg, l_odo_theta_deg_delta_max);

      printf ("MOVE : %s %f m/s\n", m_s.forward_move?"FORWARDS":"BACKWARDS", m_s.speed_abs);
      printf ("robot_sensors = %.8x\n", my_robot_sensors);

      printf ("\n");
#endif
      my_robot_sensors = my_robot_sensors; /* avoid stupid compiler message */
      dbg_cnt++;
    } /* (m_s.local_ts_ms!=l_uart_thread_time_ms_old) */
    else
    {

#ifndef WIN32
      usleep(10000);
#else
      Sleep(10);
#endif
    }

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
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

