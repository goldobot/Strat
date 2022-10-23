#ifdef WIN32
#include <windows.h>
#endif

#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#include "world_state.hpp"


using namespace goldobot;


WorldState WorldState::s_instance;

WorldState& WorldState::instance()
{
  return s_instance;
}

WorldState::WorldState()
{
  strncpy(m_thread_name,"WorldState",sizeof(m_thread_name));

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
  memset (&m_s, 0, sizeof(m_s));

  m_s.n_observ = 1;
  strncpy(m_s.observable[0].name, "BLUE_OBJ", sizeof (m_s.observable[0].name)-1);
  m_s.observable[0].x_mm  = 0;
  m_s.observable[0].y_mm  = 0;
  m_s.observable[0].value = false;

  m_s.n_detected_objects = 0;

  pthread_mutex_init(&m_lock, NULL);

  return 0;
}

void WorldState::taskFunction()
{
  unsigned int time_ms = 0;
  struct timespec my_tp;

  m_task_running = true;

  while(!m_stop_task)
  {
    clock_gettime(1, &my_tp);

    time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
    m_s.local_ts_ms = time_ms;

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

  for (int i=0; i<m_s.n_observ; i++)
  {
    if (strcmp(m_s.observable[i].name,observable_name)==0)
    {
      val = m_s.observable[i].value;
      break;
    }
  }

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

int WorldState::read_yaml_conf(const char *)
{
  /* bouchon */
  return 0;
}

void WorldState::start_signal()
{
  /* bouchon */
}

void WorldState::sim_send_robot_detection()
{
  /* bouchon */
}

void WorldState::lock_update(bool disable)
{
  /* bouchon */
  disable = disable;
}

