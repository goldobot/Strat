#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#define ROBOT_SIM 1

#define SIM_GRANULARITY 10000

#include "robot_state.hpp"
#include "comm_zmq.hpp"
#include "sim/virtual_robot.hpp"


using namespace goldobot;


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

  VirtualRobot::myself().set_enable(true);
  VirtualRobot::myself().set_autom(false);

  /* FIXME : TODO : finish init of virtual robots and load conf */
  //VirtualRobot::myself().sv().p.x = 1.0; /* FIXME : TODO :remove after dbg! */
  //VirtualRobot::myself().sv().p.y = 0.0; /* FIXME : TODO :remove after dbg! */
  VirtualRobot::myself().sv().p.x = 0.8; /* FIXME : TODO :remove after dbg! */
  VirtualRobot::myself().sv().p.y =-1.4; /* FIXME : TODO :remove after dbg! */
  VirtualRobot::myself().sv().theta= M_PI/2;/* FIXME : TODO:remove after dbg! */

  VirtualRobot::partner().set_enable(false);
  VirtualRobot::partner().set_autom(true);

  VirtualRobot::adversary1().set_enable(false);
  VirtualRobot::adversary1().set_autom(true);

  VirtualRobot::adversary2().set_enable(false);
  VirtualRobot::adversary2().set_autom(true);

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

    /**  Simulate the robots  *************************************************/

    for (int i=0; i<SIM_GRANULARITY; i++)
    {
      double delta_time_s = delta_time_ms/1000.0;
      VirtualRobot::myself().sim_update(delta_time_s/SIM_GRANULARITY);
    }

    /**  Update the robot(s) state(s)  ****************************************/

    lock();
    m_s.x_mm          = VirtualRobot::myself().sv().p.x*1000.0;
    m_s.y_mm          = VirtualRobot::myself().sv().p.y*1000.0;
    m_s.theta_deg     = VirtualRobot::myself().sv().theta*180.0/M_PI;
    m_s.robot_sensors = VirtualRobot::myself().gpio();
    release();

    /**  Send the new state information to the HMI  ***************************/

    /* FIXME : TODO : define method.. */
    {
      unsigned char msg_buf[64];

      /* FIXME : TODO : use defines.. */
      unsigned short int msg_code = 0x0001; /*Heartbeat*/
      unsigned char *_pc = msg_buf;
      int msg_buf_len = 0;
      int field_len = 0;

      field_len = sizeof(unsigned short);
      memcpy (_pc, (unsigned char *)&msg_code, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* timestamp */
      int msg_timestamp = time_ms; /* FIXME : TODO */
      field_len = sizeof(int);
      memcpy (_pc, (unsigned char *)&msg_timestamp, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      CommZmq::instance().send(msg_buf, msg_buf_len, 0);
    }

    /* FIXME : TODO : define method.. */
    {
      unsigned char msg_buf[64];

      /* FIXME : TODO : use defines.. */
      unsigned short int msg_code = 0x0008; /*PropulsionTelemetry*/
      unsigned char *_pc = msg_buf;
      int msg_buf_len = 0;
      int field_len = 0;

      field_len = sizeof(unsigned short);
      memcpy (_pc, (unsigned char *)&msg_code, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* x */
      short msg_x = VirtualRobot::myself().sv().p.x * 4000.0;
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_x, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* y */
      short msg_y = VirtualRobot::myself().sv().p.y * 4000.0;
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_y, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* yaw */
      short msg_yaw = 32767.0 * VirtualRobot::myself().sv().theta / M_PI;
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_yaw, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* speed */
      double _v_x = VirtualRobot::myself().sv().v.x;
      double _v_y = VirtualRobot::myself().sv().v.y;
      double _v_abs = sqrt(_v_x*_v_x+_v_y*_v_y);
      short msg_speed = _v_abs * 1000.0;
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_speed, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* yaw_rate */
      short msg_yaw_rate = fabs(VirtualRobot::myself().sv().v_theta) * 1000.0;
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_yaw_rate, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* acceleration */
      short msg_acceleration = 0; /* FIXME : TODO */
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_acceleration, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* angular_acceleration */
      short msg_angular_acceleration = 0; /* FIXME : TODO */
      field_len = sizeof(short);
      memcpy (_pc, (unsigned char *)&msg_angular_acceleration, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* left_encoder */
      unsigned short msg_left_encoder = 0; /* FIXME : TODO */
      field_len = sizeof(unsigned short);
      memcpy (_pc, (unsigned char *)&msg_left_encoder, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* right_encoder */
      unsigned short msg_right_encoder = 0; /* FIXME : TODO */
      field_len = sizeof(unsigned short);
      memcpy (_pc, (unsigned char *)&msg_right_encoder, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* left_pwm */
      char msg_left_pwm = 0; /* FIXME : TODO */
      field_len = sizeof(char);
      memcpy (_pc, (unsigned char *)&msg_left_pwm, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* right_pwm */
      char msg_right_pwm = 0; /* FIXME : TODO */
      field_len = sizeof(char);
      memcpy (_pc, (unsigned char *)&msg_right_pwm, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* state */
      unsigned char msg_state = 0; /* FIXME : TODO */
      field_len = sizeof(unsigned char);
      memcpy (_pc, (unsigned char *)&msg_state, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      /* error */
      unsigned char msg_error = 0; /* FIXME : TODO */
      field_len = sizeof(unsigned char);
      memcpy (_pc, (unsigned char *)&msg_error, field_len);
      _pc += field_len;
      msg_buf_len += field_len;

      CommZmq::instance().send(msg_buf, msg_buf_len, 0);
    }

    /**  Iteration end in the main loop of the simulation  thread  ************/

    usleep (10000);
    pthread_yield();
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

void RobotState::sim_send(const unsigned char *msg_buf, size_t msg_len)
{
  VirtualRobot::myself().sim_receive(msg_buf, msg_len);
}

void RobotState::sim_recv(unsigned char *msg_buf, size_t msg_len)
{
  /* bouchon */
  msg_buf = msg_buf;
  msg_len = msg_len;
}
