#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_thread.hpp"

namespace goldobot
{
  typedef struct _robot_state_info {
    unsigned int local_ts_ms; /* was g_odo_thread_time_ms */
    unsigned int remote_ts_ms; /* g_odo_time_ms */
    int          x_mm;
    int          y_mm;
    double       theta_deg;
    double       speed_abs;
    bool         forward_move;
    unsigned int robot_sensors;
  } robot_state_info_t;

  class RobotState : public GoldoThread
  {
  public:
    static RobotState& instance();

    RobotState();

    int init();

    virtual void taskFunction();

    /* FIXME : TODO : refactor (dangerous!) */
    robot_state_info_t& s() {return m_s;}

    void get_s(robot_state_info_t *_s);

    void set_s(robot_state_info_t *_s);

    bool tirette_present() 
      {
        return (m_s.robot_sensors&GPIO_START_MASK)!=0;
      }

    bool propulsion_busy() 
      {
        return (m_s.robot_sensors&FLAG_PROPULSION_BUSY_MASK)!=0;
      }

    bool propulsion_error() 
      {
        return (m_s.robot_sensors&FLAG_PROPULSION_ERROR_MASK)!=0;
      }

    int lock(int timeout_ms = -1);

    void release();

    static const unsigned int GPIO_GREEN_LED_MASK          = 0x00000001;
    static const unsigned int GPIO_START_MASK              = 0x00000002;
    static const unsigned int GPIO_EMERGENCY_STOP_MASK     = 0x00000004;
    static const unsigned int GPIO_DYNAMIXEL_DIR_MASK      = 0x00000008;
    static const unsigned int GPIO_SIDE_SELECT_MASK        = 0x00000010;
    static const unsigned int GPIO_AUTOCONFIG_MASK         = 0x00000020;

    static const unsigned int FLAG_PROPULSION_BUSY_MASK    = 0x00010000;
    static const unsigned int FLAG_PROPULSION_ERROR_MASK   = 0x00020000;

  private:
    robot_state_info_t m_s;

    pthread_mutex_t m_lock;

    static RobotState s_instance;
  };
}

