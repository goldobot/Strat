#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_thread.hpp"

namespace goldobot
{
  class RobotStrat : public GoldoThread
  {
  public:
    static RobotStrat& instance();

    RobotStrat();

    int init(char *strat_file_name);

    virtual void taskFunction();

    void start_match();

    /* FIXME : TODO */

  private:
    char m_strat_file_name[40];

    unsigned char m_nucleo_cmd_buf[1024];

    bool m_dbg_start_match;

    static RobotStrat s_instance;
  };
}

