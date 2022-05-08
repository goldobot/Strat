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

  m_hard_obstacles_cnt = 0;
  memset (&m_hard_obstacles, 0, sizeof(m_hard_obstacles));
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

  /* FIXME : TODO : put these in conf .. */
  m_hard_obstacles_cnt = 1;
  goldo_segm_2d_t _hard_obstacles[] = {
    {{0.000, -0.010}, {0.250,  0.010}}, 
  };
  memcpy ((unsigned char *)m_hard_obstacles, (unsigned char *)_hard_obstacles, 
          sizeof(_hard_obstacles));

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
  YAML::Node myself_node = yconf["myself"];
  if (!myself_node) 
  {
    printf ("ERROR : no 'myself' section\n");
    return -1;
  }
  if(VirtualRobots::myself().read_yaml_conf(myself_node)!=0) 
  {
    printf ("ERROR : simul conf read failed\n");
    return -1;
  }

  /* Load simul task for 'partner' */
  YAML::Node partner_node = yconf["partner"];
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
    printf ("WARNING : no 'partner' section\n");
  }

  /* Load simul task for 'adversary1' */
  YAML::Node adversary1_node = yconf["adversary1"];
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
    printf ("WARNING : no 'adversary1' section\n");
  }

  /* Load simul task for 'adversary2' */
  YAML::Node adversary2_node = yconf["adversary2"];
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
    printf ("WARNING : no 'adversary2' section\n");
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
  volatile unsigned int time_ms = 0;
  volatile unsigned int time_ms_old = 0;
  int delta_time_ms = 0;
  int start_match_time_ms = 0;
  int current_match_time_ms = 0;
  struct timespec my_tp;

  unsigned int telemetry_ex_cnt = 0;

  m_task_running = true;

  while(!m_stop_task)
  {
    /**  Time  ****************************************************************/

    clock_gettime(1, &my_tp);

    time_ms_old = time_ms;
    time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
    m_s.local_ts_ms = time_ms;
    if (time_ms_old==0) /* avoid annoying artefact.. */
    {
      delta_time_ms = 0;
    }
    else
    {
      delta_time_ms = time_ms - time_ms_old;
    }

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
      double delta_time_s = (double)delta_time_ms/1000.0;

      if (!m_update_lock) {
        VirtualRobots::myself().sim_update(delta_time_s/SIM_GRANULARITY);
      }
      VirtualRobots::partner().sim_update(delta_time_s/SIM_GRANULARITY);
      VirtualRobots::adversary1().sim_update(delta_time_s/SIM_GRANULARITY);
      VirtualRobots::adversary2().sim_update(delta_time_s/SIM_GRANULARITY);

      RobotState::instance().s().robot_sensors= VirtualRobots::myself().gpio();
    }

    /**  Update the robot(s) state(s)  ****************************************/

    robot_state_info_t& my_s = RobotState::instance().s();
    RobotState::instance().lock();
    my_s.x_mm          = VirtualRobots::myself().odometry().p.x*1000.0;
    my_s.y_mm          = VirtualRobots::myself().odometry().p.y*1000.0;
    my_s.theta_deg     = VirtualRobots::myself().odometry().theta*180.0/M_PI;
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

    /**  Simulate colisions  **************************************************/
    /* FIXME : TODO : WIP */
    {
      goldo_vec_2d_t my_p = VirtualRobots::myself().sv().p;
      goldo_vec_2d_t partner_p = VirtualRobots::partner().sv().p;
      goldo_vec_2d_t adv1_p = VirtualRobots::adversary1().sv().p;
      goldo_vec_2d_t adv2_p = VirtualRobots::adversary2().sv().p;

      for (int i=0; i<m_hard_obstacles_cnt; i++)
      {
        if ((!VirtualRobots::myself().is_crashed()) && 
            (goldo_segm_dist(my_p,m_hard_obstacles[i])<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Our robot crashed!!\n");
#if 1
          printf ("  my_p = <%2.6f,%2.6f>\n", my_p.x, my_p.y);
          printf ("  i = %d\n", i);
          printf ("  obstacles[i].p1 = <%2.6f,%2.6f>\n", 
                  m_hard_obstacles[i].p1.x, m_hard_obstacles[i].p1.y);
          printf ("  obstacles[i].p2 = <%2.6f,%2.6f>\n", 
                  m_hard_obstacles[i].p2.x, m_hard_obstacles[i].p2.y);
#endif
          VirtualRobots::myself().sim_crash_robot();
        }
        if ((!VirtualRobots::partner().is_crashed_or_disabled()) && 
            (goldo_segm_dist(partner_p,m_hard_obstacles[i])<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Partner robot crashed.\n");
          VirtualRobots::partner().sim_crash_robot();
        }
        if ((!VirtualRobots::adversary1().is_crashed_or_disabled()) && 
            (goldo_segm_dist(adv1_p,m_hard_obstacles[i])<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Adversary1 robot crashed.\n");
          VirtualRobots::adversary1().sim_crash_robot();
        }
        if ((!VirtualRobots::adversary2().is_crashed_or_disabled()) && 
            (goldo_segm_dist(adv2_p,m_hard_obstacles[i])<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Adversary2 robot crashed.\n");
          VirtualRobots::adversary2().sim_crash_robot();
        }
      }

      if ((!VirtualRobots::myself().is_crashed()))
      {
        if ((VirtualRobots::partner().enabled()) && (goldo_dist(my_p,partner_p)<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Collision with partner. Our robot crashed!!\n");
          VirtualRobots::myself().sim_crash_robot();
          VirtualRobots::partner().sim_crash_robot();
        }
        if ((VirtualRobots::adversary1().enabled()) && (goldo_dist(my_p,adv1_p)<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Collision with adversary1. Our robot crashed!!\n");
          VirtualRobots::myself().sim_crash_robot();
          VirtualRobots::adversary1().sim_crash_robot();
        }
        if ((VirtualRobots::adversary2().enabled()) && (goldo_dist(my_p,adv2_p)<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Collision with adversary2. Our robot crashed!!\n");
          VirtualRobots::myself().sim_crash_robot();
          VirtualRobots::adversary2().sim_crash_robot();
        }
      }

      if ((!VirtualRobots::partner().is_crashed_or_disabled()))
      {
        if ((VirtualRobots::adversary1().enabled()) && (goldo_dist(partner_p,adv1_p)<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Collision between partner and adversary1\n");
          VirtualRobots::partner().sim_crash_robot();
          VirtualRobots::adversary1().sim_crash_robot();
        }
        if ((VirtualRobots::adversary2().enabled()) && (goldo_dist(partner_p,adv2_p)<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Collision between partner and adversary2\n");
          VirtualRobots::partner().sim_crash_robot();
          VirtualRobots::adversary2().sim_crash_robot();
        }
      }

      if ((!VirtualRobots::adversary1().is_crashed_or_disabled()))
      {
        if ((VirtualRobots::adversary2().enabled()) && (goldo_dist(adv1_p,adv2_p)<SIM_CRASH_DIST))
        {
          printf ("DEBUG : Collision between adversary1 and adversary2\n");
          VirtualRobots::adversary1().sim_crash_robot();
          VirtualRobots::adversary2().sim_crash_robot();
        }
      }
    }

    /**  Send the new state information to the HMI  ***************************/

    //sim_send_heartbeat(time_ms);
    VirtualRobots::myself().sim_send_heartbeat(current_match_time_ms);

    VirtualRobots::myself().sim_send_propulsion_telemetry();

    if ((telemetry_ex_cnt%5)==0)
    {
      VirtualRobots::myself().sim_send_propulsion_telemetry_ex();
    }
    telemetry_ex_cnt++;

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

detected_object_info_t& WorldState::detected_object(int _obj_idx)
{
  /* FIXME : TODO : throw exception if _obst_idx out of bounds */
  return m_s.detected_object[_obj_idx];
}

bool WorldState::get_observable_value(char *observable_name)
{
  bool val = false;

#if 0 /* FIXME : TODO : polling des differents observables dans taskFunction() */
  for (int i=0; i<n_observ; i++)
  {
    if (strcmp(m_s.observable[i].name,observable_name)==0)
    {
      val = m_s.observable[i].value;
      break;
    }
  }
#else
  if (strcmp(m_s.observable[0].name,observable_name)==0)
  {
    int goldo_girouette_fd;
    char read_buf[8];
    int res;

    read_buf[0] = 0;

    goldo_girouette_fd = open ("/tmp/girouette.txt", O_RDWR);
    if (goldo_girouette_fd<0) {
      printf ("error opening /tmp/girouette.txt\n");
      return false;
    }
    res = read (goldo_girouette_fd,read_buf,1);
    if (res<0) {
      printf ("error reading girouette value\n");
    }

    if (read_buf[0]=='S') val=true;

    close (goldo_girouette_fd);
  }
#endif

  return val;
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


void WorldState::sim_send_robot_detection()
{
  robot_detection_msg_t my_message;

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

    CommZmq::instance().send_robot_detection((const char*)(&my_message), sizeof(my_message));
  }
}

void WorldState::lock_update(bool lock_state)
{
  m_update_lock = lock_state;
}

