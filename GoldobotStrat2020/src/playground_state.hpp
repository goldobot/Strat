#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_thread.hpp"
#include "detect/robot_detection_info.hpp"

namespace goldobot
{
  typedef struct _playground_state_info {
    int                   n_detected_robots;
    detected_robot_info_t detected_robot[3];
  } playground_state_info_t;

  class PlaygroundState : public GoldoThread
  {
  public:
    static PlaygroundState& instance();

    PlaygroundState();

    int init();

    virtual void taskFunction();

    /* FIXME : TODO : refactor (dangerous!) */
    playground_state_info_t& s() {return m_s;}

    detected_robot_info_t& detected_robot(int _obst_idx);

    int lock(int timeout_ms = -1);

    void release();

  private:
    playground_state_info_t m_s;

    /* FIXME : TODO : refactor (factorise) */
    pthread_mutex_t m_lock;

    static PlaygroundState s_instance;
  };
}

