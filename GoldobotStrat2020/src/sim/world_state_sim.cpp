#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <zmq.h>
#include <fstream>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#define ROBOT_SIM 1

#define SIM_GRANULARITY 10000

#include "goldo_conf.hpp"
#include "robot_state.hpp"
#include "world_state.hpp"
#include "comm_zmq.hpp"
#include "sim/virtual_robot.hpp"


using namespace goldobot;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

WorldState WorldState::s_instance;

WorldState& WorldState::instance()
{
  return s_instance;
}

WorldState::WorldState()
{
  strncpy(m_thread_name,"WorldState_sim",sizeof(m_thread_name));

  memset (&m_s, 0, sizeof(m_s));

  m_stop_task = false;
  m_task_running = false;

  pthread_mutex_init(&m_lock, NULL);

  m_match_started = false;
}

int WorldState::init()
{
  m_match_started = false;

  memset (&m_s, 0, sizeof(m_s));

  pthread_mutex_init(&m_lock, NULL);

  VirtualRobots::myself().set_enable(true);
  VirtualRobots::myself().set_autom(false);

  VirtualRobots::partner().set_enable(false);
  VirtualRobots::partner().set_autom(true);

  VirtualRobots::adversary1().set_enable(false);
  VirtualRobots::adversary1().set_autom(true);

  VirtualRobots::adversary2().set_enable(false);
  VirtualRobots::adversary2().set_autom(true);

  const char *sim_fname = GoldoConf::instance().c().conf_simul_file_str;
  read_yaml_conf (sim_fname);

  return 0;
}

int WorldState::read_yaml_conf(const char *fname)
{
  std::ifstream fin;
  YAML::Node yconf;

  try 
  {
    fin.open(fname);
    yconf = YAML::Load(fin);
  } 
  catch(const YAML::Exception& e)
  {
    printf ("ERROR : %s\n", e.what());
    return -1;
  }

  /* Load simul task for 'myself' */
  YAML::Node myself_node = yconf["myself_sim_task"];
  if (!myself_node) 
  {
    printf ("ERROR : no 'myself_sim_task' section\n");
    return -1;
  }
  if(VirtualRobots::myself().read_yaml_conf(myself_node)!=0) 
  {
    printf ("ERROR : simul conf read failed\n");
    return -1;
  }

  /* Load simul task for 'partner' */
  YAML::Node partner_node = yconf["partner_sim_task"];
  if (partner_node) 
  {
    if(VirtualRobots::partner().read_yaml_conf(partner_node)==0) 
    {
      /* FIXME : TODO : define explicit 'enable' flag in conf */
      VirtualRobots::partner().set_enable(true);
    }
    else
    {
      printf ("WARNING : invalid partner simul conf\n");
    }
  }
  else
  {
    printf ("WARNING : no 'partner_sim_task' section\n");
  }

  /* Load simul task for 'adversary1' */
  YAML::Node adversary1_node = yconf["adversary1_sim_task"];
  if (adversary1_node) 
  {
    if(VirtualRobots::adversary1().read_yaml_conf(adversary1_node)==0) 
    {
      /* FIXME : TODO : define explicit 'enable' flag in conf */
      VirtualRobots::adversary1().set_enable(true);
    }
    else
    {
      printf ("WARNING : invalid adversary1 simul conf\n");
    }
  }
  else
  {
    printf ("WARNING : no 'adversary1_sim_task' section\n");
  }

  /* Load simul task for 'adversary2' */
  YAML::Node adversary2_node = yconf["adversary2_sim_task"];
  if (adversary2_node) 
  {
    if(VirtualRobots::adversary2().read_yaml_conf(adversary2_node)==0) 
    {
      /* FIXME : TODO : define explicit 'enable' flag in conf */
      VirtualRobots::adversary2().set_enable(true);
    }
    else
    {
      printf ("WARNING : invalid adversary2 simul conf\n");
    }
  }
  else
  {
    printf ("WARNING : no 'adversary2_sim_task' section\n");
  }

  return 0;
}

void WorldState::start_signal()
{
  VirtualRobots::partner().create_me_list_from_strat();
  VirtualRobots::adversary1().create_me_list_from_strat();
  VirtualRobots::adversary2().create_me_list_from_strat();

  m_match_started = true;
}

void WorldState::taskFunction()
{
  unsigned int time_ms = 0;
  unsigned int time_ms_old = 0;
  int delta_time_ms = 0;
  int start_match_time_ms = 0;
  int current_match_time_ms = 0;
  struct timespec my_tp;

  m_task_running = true;

  while(!m_stop_task)
  {
    /**  Time  ****************************************************************/

    clock_gettime(1, &my_tp);

    time_ms_old = time_ms;
    time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
    m_s.local_ts_ms = time_ms;
    delta_time_ms = time_ms - time_ms_old;

    /* FIXME : TODO : refactor : crap! */
    if (m_match_started)
    {
      current_match_time_ms = time_ms - start_match_time_ms;
    }
    else
    {
      start_match_time_ms = time_ms;
    }

    /**  Simulate the robots  *************************************************/

    for (int i=0; i<SIM_GRANULARITY; i++)
    {
      double delta_time_s = delta_time_ms/1000.0;
      VirtualRobots::myself().sim_update(delta_time_s/SIM_GRANULARITY);
      VirtualRobots::partner().sim_update(delta_time_s/SIM_GRANULARITY);
      VirtualRobots::adversary1().sim_update(delta_time_s/SIM_GRANULARITY);
      VirtualRobots::adversary2().sim_update(delta_time_s/SIM_GRANULARITY);
    }

    /**  Update the robot(s) state(s)  ****************************************/

    robot_state_info_t& my_s = RobotState::instance().s();
    RobotState::instance().lock();
    my_s.x_mm          = VirtualRobots::myself().sv().p.x*1000.0;
    my_s.y_mm          = VirtualRobots::myself().sv().p.y*1000.0;
    my_s.theta_deg     = VirtualRobots::myself().sv().theta*180.0/M_PI;
    my_s.robot_sensors = VirtualRobots::myself().gpio();
    RobotState::instance().release();

    /* FIXME : TODO : refactor (not clean..) */
    lock();
    int r=0;
    if (VirtualRobots::partner().enabled()) 
    {
      m_s.detected_robot[r].timestamp_ms = time_ms;
      m_s.detected_robot[r].id = 0;
      m_s.detected_robot[r].x_mm = VirtualRobots::partner().sv().p.x*1000.0;
      m_s.detected_robot[r].y_mm = VirtualRobots::partner().sv().p.y*1000.0;
      m_s.detected_robot[r].vx_mm_sec = 
        VirtualRobots::partner().sv().v.x*1000.0;
      m_s.detected_robot[r].vy_mm_sec = 
        VirtualRobots::partner().sv().v.y*1000.0;
      m_s.detected_robot[r].ax_mm_sec_2 = 0; /* FIXME : TODO */
      m_s.detected_robot[r].ay_mm_sec_2 = 0; /* FIXME : TODO */
      m_s.detected_robot[r].detect_quality = 40;
      r++;
    }
    if (VirtualRobots::adversary1().enabled()) 
    {
      m_s.detected_robot[r].timestamp_ms = time_ms;
      m_s.detected_robot[r].id = 1;
      m_s.detected_robot[r].x_mm = VirtualRobots::adversary1().sv().p.x*1000.0;
      m_s.detected_robot[r].y_mm = VirtualRobots::adversary1().sv().p.y*1000.0;
      m_s.detected_robot[r].vx_mm_sec = 
        VirtualRobots::adversary1().sv().v.x*1000.0;
      m_s.detected_robot[r].vy_mm_sec = 
        VirtualRobots::adversary1().sv().v.y*1000.0;
      m_s.detected_robot[r].ax_mm_sec_2 = 0; /* FIXME : TODO */
      m_s.detected_robot[r].ay_mm_sec_2 = 0; /* FIXME : TODO */
      m_s.detected_robot[r].detect_quality = 30;
      r++;
    }
    if (VirtualRobots::adversary2().enabled()) 
    {
      m_s.detected_robot[r].timestamp_ms = time_ms;
      m_s.detected_robot[r].id = 2;
      m_s.detected_robot[r].x_mm = VirtualRobots::adversary2().sv().p.x*1000.0;
      m_s.detected_robot[r].y_mm = VirtualRobots::adversary2().sv().p.y*1000.0;
      m_s.detected_robot[r].vx_mm_sec = 
        VirtualRobots::adversary2().sv().v.x*1000.0;
      m_s.detected_robot[r].vy_mm_sec = 
        VirtualRobots::adversary2().sv().v.y*1000.0;
      m_s.detected_robot[r].ax_mm_sec_2 = 0; /* FIXME : TODO */
      m_s.detected_robot[r].ay_mm_sec_2 = 0; /* FIXME : TODO */
      m_s.detected_robot[r].detect_quality = 20;
      r++;
    }
    m_s.n_detected_robots = r;
    release();

    /**  Send the new state information to the HMI  ***************************/

    //sim_send_heartbeat(time_ms);
    sim_send_heartbeat(current_match_time_ms);

    sim_send_propulsion_telemetry();

    sim_send_robot_detection();

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
  } /* while(!m_stop_task) */

  m_task_running = false;
}

detected_robot_info_t& WorldState::detected_robot(int _obst_idx)
{
  /* FIXME : TODO : throw exception if _obst_idx out of bounds */
  return m_s.detected_robot[_obst_idx];
}

int WorldState::lock(int timeout_ms)
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

void WorldState::release()
{
  pthread_mutex_unlock(&m_lock);
}


void WorldState::sim_send_heartbeat(int time_ms)
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

void WorldState::sim_send_propulsion_telemetry()
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
  short msg_x = VirtualRobots::myself().sv().p.x * 4000.0;
  field_len = sizeof(short);
  memcpy (_pc, (unsigned char *)&msg_x, field_len);
  _pc += field_len;
  msg_buf_len += field_len;

  /* y */
  short msg_y = VirtualRobots::myself().sv().p.y * 4000.0;
  field_len = sizeof(short);
  memcpy (_pc, (unsigned char *)&msg_y, field_len);
  _pc += field_len;
  msg_buf_len += field_len;

  /* yaw */
  short msg_yaw = 32767.0 * VirtualRobots::myself().sv().theta / M_PI;
  field_len = sizeof(short);
  memcpy (_pc, (unsigned char *)&msg_yaw, field_len);
  _pc += field_len;
  msg_buf_len += field_len;

  /* speed */
  double _v_x = VirtualRobots::myself().sv().v.x;
  double _v_y = VirtualRobots::myself().sv().v.y;
  double _v_abs = sqrt(_v_x*_v_x+_v_y*_v_y);
  short msg_speed = _v_abs * 1000.0;
  field_len = sizeof(short);
  memcpy (_pc, (unsigned char *)&msg_speed, field_len);
  _pc += field_len;
  msg_buf_len += field_len;

  /* yaw_rate */
  short msg_yaw_rate = fabs(VirtualRobots::myself().sv().v_theta) * 1000.0;
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

void WorldState::sim_send_robot_detection()
{
  unsigned short my_message_type;
  robot_detection_msg_t my_message;

  my_message_type = 1280; /* FIXME : TODO : use mesage_types.hpp */

  for (int r=0; r<m_s.n_detected_robots; r++)
  {
    my_message.timestamp_ms   = m_s.detected_robot[r].timestamp_ms;
    my_message.id             = m_s.detected_robot[r].id;
    my_message.x_mm_X4        = m_s.detected_robot[r].x_mm * 4.0;
    my_message.y_mm_X4        = m_s.detected_robot[r].y_mm * 4.0;
    my_message.vx_mm_sec      = m_s.detected_robot[r].vx_mm_sec;
    my_message.vy_mm_sec      = m_s.detected_robot[r].vy_mm_sec;
    my_message.ax_mm_sec_2    = m_s.detected_robot[r].ax_mm_sec_2;
    my_message.ay_mm_sec_2    = m_s.detected_robot[r].ay_mm_sec_2;
    my_message.detect_quality = m_s.detected_robot[r].detect_quality;

    CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
    CommZmq::instance().send((const char*)(&my_message), sizeof(my_message), 0);
  }
}

