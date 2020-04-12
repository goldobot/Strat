#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_thread.hpp"

namespace goldobot
{
  class OdometryState : public GoldoThread
  {
  public:
    static OdometryState& instance();

    OdometryState();

    int init();

    virtual void taskFunction();

    int lock();

    void release();

    /* FIXME : TODO : ok to let these fields in public access?*/
    unsigned int m_local_ts_ms; /* was g_odo_thread_time_ms */
    unsigned int m_remote_ts_ms; /* g_odo_time_ms */
    unsigned int m_x_mm;
    unsigned int m_y_mm;
    double       m_theta_deg;
    double       m_speed_abs;
    bool         m_forward_move;
    unsigned int m_robot_sensors;

  private:
    static OdometryState s_instance;
  };
}

