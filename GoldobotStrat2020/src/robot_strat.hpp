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

    static RobotStrat s_instance;
  };
}

