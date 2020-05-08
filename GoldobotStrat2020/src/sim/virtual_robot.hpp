#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "strat/robot_strat_types.hpp"

#ifndef ROBOT_SIM
#error "You must define ROBOT_SIM"
#endif

namespace goldobot
{

  class VirtualRobot {
  public:
    VirtualRobot();

    void sim_receive(const unsigned char *msg_buf, size_t msg_len);

    /* FIXME : TODO */
  private:
    void on_cmd_execute_trajectory(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_execute_point_to(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_set_pose(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_start_sequence(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_propulsion_clear_error(unsigned char *msg_buf, size_t msg_len);

    unsigned char m_recv_buf[256];
    static const int RECV_BUF_SZ = sizeof(m_recv_buf);
  };

}
