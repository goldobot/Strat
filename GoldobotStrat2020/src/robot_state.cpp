#include <string.h>
#include <unistd.h>
#include <math.h>

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

  m_local_ts_ms  = 0;
  m_remote_ts_ms = 0;
  m_x_mm         = 0;
  m_y_mm         = 0;
  m_theta_deg    = 0.0;

  m_speed_abs = 0.0;
  m_forward_move = true;

  m_robot_sensors = 0;
}

int RobotState::init()
{
  m_local_ts_ms  = 0;
  m_remote_ts_ms = 0;
  m_x_mm         = 0;
  m_y_mm         = 0;
  m_theta_deg    = 0.0;

  m_speed_abs = 0.0;
  m_forward_move = true;

  m_robot_sensors = 0;

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


  m_speed_abs = 0.0;
  m_forward_move = true;

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
    if (RobotState::instance().m_local_ts_ms!=l_uart_thread_time_ms_old) 
    {
      struct timespec my_tp;
      unsigned int dbg_thread_time_ms;

      RobotState::instance().lock();
      volatile unsigned int my_uart_thread_time_ms = 
        RobotState::instance().m_local_ts_ms;
      volatile unsigned int my_odo_time_ms = 
        RobotState::instance().m_remote_ts_ms;
      volatile int    my_odo_x_mm = RobotState::instance().m_x_mm;
      volatile int    my_odo_y_mm = RobotState::instance().m_y_mm;
      volatile double my_odo_theta_deg = 
        RobotState::instance().m_theta_deg;
      volatile unsigned int my_robot_sensors = 
        RobotState::instance().m_robot_sensors;
      RobotState::instance().release();

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

        if (l_odo_time_ms_delta==0) m_speed_abs = 0.0;
        else m_speed_abs = ((double)l_odo_d_mm_delta/*/1000.0*/)/(l_odo_time_ms_delta/*/1000.0*/); // result in m/sec
        if (m_speed_abs>0.005) 
        {
          m_forward_move = ((l_odo_x_mm_delta*cos(l_odo_theta_rad) + l_odo_y_mm_delta*sin(l_odo_theta_rad)) > 0.0);
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

      printf ("MOVE : %s %f m/s\n", m_forward_move?"FORWARDS":"BACKWARDS", m_speed_abs);
      printf ("robot_sensors = %.8x\n", my_robot_sensors);

      printf ("\n");
#endif
      my_robot_sensors = my_robot_sensors; /* avoid stupid compiler message */
      dbg_cnt++;
    } /* (RobotState::instance().m_local_ts_ms!=l_uart_thread_time_ms_old) */
    else
    {
      usleep (10000);
    }

    pthread_yield();
  } /* while(!m_stop_task) */

  m_task_running = false;
}

int RobotState::lock()
{
  /* FIXME : TODO */

  return 0;
}

void RobotState::release()
{
  /* FIXME : TODO */
}

