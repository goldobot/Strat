#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_geometry.hpp"
#include "goldo_thread.hpp"
#include "detect/robot_detection_info.hpp"

namespace goldobot
{
  typedef struct _environment_observable_bool {
    char name[64];
    double x_mm;
    double y_mm;
    bool value;
  } environment_observable_bool_t;

  typedef struct _detected_object_info {
    unsigned int timestamp_ms;
    unsigned int type;
    unsigned int attr;
    double x_mm;
    double y_mm;
    unsigned int detect_quality;
  } detected_object_info_t;

  typedef struct _world_state_info {
    unsigned int                    local_ts_ms;
    int                             n_detected_robots;
    detected_robot_info_t           detected_robot[3];
    int                             n_detected_objects;
    detected_object_info_t          detected_object[10];
    int                             n_observ;
    environment_observable_bool_t   observable[4];
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

    detected_object_info_t& detected_object(int _obj_idx);

    bool get_observable_value(char *observable_name);

    int lock(int timeout_ms = -1);

    void release();

    /* used in simulation */
    void start_signal();

    void lock_update(bool lock_state);

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
    void sim_send_robot_detection();

    bool m_update_lock{false};

    static WorldState s_instance;
  };
}

