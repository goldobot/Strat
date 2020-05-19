#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_geometry.hpp"
#include "goldo_thread.hpp"
#include "detect/robot_detection_info.hpp"

namespace goldobot
{
  typedef struct _world_state_info {
    unsigned int          local_ts_ms;
    int                   n_detected_robots;
    detected_robot_info_t detected_robot[3];
  } world_state_info_t;

  class WorldState : public GoldoThread
  {
  public:
    static WorldState& instance();

    WorldState();

    int init();

    int read_yaml_conf(const char *fname);

    virtual void taskFunction();

    /* FIXME : TODO : refactor (dangerous!) */
    world_state_info_t& s() {return m_s;}

    detected_robot_info_t& detected_robot(int _obst_idx);

    int lock(int timeout_ms = -1);

    void release();

    /* used in simulation */
    void start_signal();

  private:
    world_state_info_t m_s;

    /* FIXME : TODO : refactor (factorise) */
    pthread_mutex_t m_lock;

    /* used in simulation */
    bool m_match_started;

    /* used in simulation */
    int m_hard_obstacles_cnt;
    goldo_segm_2d_t m_hard_obstacles[64];
    static constexpr double SIM_CRASH_DIST = 0.050;

    /* used in simulation */
    void sim_send_heartbeat(int time_ms);
    /* used in simulation */
    void sim_send_propulsion_telemetry();
    /* used in simulation */
    void sim_send_robot_detection();

    static WorldState s_instance;
  };
}

