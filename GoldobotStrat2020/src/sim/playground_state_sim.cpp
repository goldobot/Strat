#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#include "playground_state.hpp"


using namespace goldobot;


PlaygroundState PlaygroundState::s_instance;

PlaygroundState& PlaygroundState::instance()
{
  return s_instance;
}

PlaygroundState::PlaygroundState()
{
  strncpy(m_thread_name,"PlaygroundState_sim",sizeof(m_thread_name));

  memset (&m_s, 0, sizeof(m_s));

  m_stop_task = false;
  m_task_running = false;

  pthread_mutex_init(&m_lock, NULL);
}

int PlaygroundState::init()
{
  memset (&m_s, 0, sizeof(m_s));

  pthread_mutex_init(&m_lock, NULL);

  return 0;
}

void PlaygroundState::taskFunction()
{

  m_task_running = true;

  while(!m_stop_task)
  {
    /* FIXME : TODO */

    usleep (10000);

    pthread_yield();
  } /* while(!m_stop_task) */

  m_task_running = false;
}

detected_robot_info_t& PlaygroundState::detected_robot(int _obst_idx)
{
  /* FIXME : TODO : throw exception if _obst_idx out of bounds */
  return m_s.detected_robot[_obst_idx];
}

int PlaygroundState::lock(int timeout_ms)
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

void PlaygroundState::release()
{
  pthread_mutex_unlock(&m_lock);
}

