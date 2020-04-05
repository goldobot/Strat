#include <string.h>
#include <unistd.h>
#include <math.h>

#include "odometry_state.hpp"


using namespace goldobot;


OdometryState OdometryState::s_instance;

OdometryState& OdometryState::instance()
{
  return s_instance;
}

OdometryState::OdometryState()
{
  m_stop_task = false;
  m_task_running = false;

  m_local_ts_ms  = 0;
  m_remote_ts_ms = 0;
  m_x_mm         = 0;
  m_y_mm         = 0;
  m_theta_deg    = 0.0;

  m_speed_abs = 0.0;
  m_forward_move = true;
}

int OdometryState::init()
{
  m_local_ts_ms  = 0;
  m_remote_ts_ms = 0;
  m_x_mm         = 0;
  m_y_mm         = 0;
  m_theta_deg    = 0.0;

  m_speed_abs = 0.0;
  m_forward_move = true;

  return 0;
}

#define MAX(a,b) ((a>b)?a:b)

void OdometryState::taskFunction()
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
    if (OdometryState::instance().m_local_ts_ms!=l_uart_thread_time_ms_old) 
    {
      struct timespec my_tp;
      unsigned int dbg_thread_time_ms;

      OdometryState::instance().lock();
      volatile unsigned int my_uart_thread_time_ms = 
        OdometryState::instance().m_local_ts_ms;
      volatile unsigned int my_odo_time_ms = 
        OdometryState::instance().m_remote_ts_ms;
      volatile int    my_odo_x_mm = OdometryState::instance().m_x_mm;
      volatile int    my_odo_y_mm = OdometryState::instance().m_y_mm;
      volatile double my_odo_theta_deg = 
        OdometryState::instance().m_theta_deg;
      OdometryState::instance().release();

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

      printf ("\n");
#endif
      dbg_cnt++;
    }
    //dbg_cnt++;


    pthread_yield();
  }

  m_task_running = false;
}

int OdometryState::lock()
{
  /* FIXME : TODO */

  return 0;
}

void OdometryState::release()
{
  /* FIXME : TODO */
}

