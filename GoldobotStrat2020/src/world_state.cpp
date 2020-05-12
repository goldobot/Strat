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
}

int WorldState::init()
{
  memset (&m_s, 0, sizeof(m_s));

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

    usleep (10000);

    pthread_yield();
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

int WorldState::read_yaml_conf(const char *)
{
  /* bouchon */
  return 0;
}

void WorldState::start_signal()
{
  /* bouchon */
}

void WorldState::sim_send_heartbeat(int time_ms)
{
  /* bouchon */
  time_ms = time_ms;
}

void WorldState::sim_send_propulsion_telemetry()
{
  /* bouchon */
}

void WorldState::sim_send_robot_detection()
{
  /* bouchon */
}


