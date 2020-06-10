#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "goldo_thread.hpp"
#include "strat/robot_strat_types.hpp"
//#include "strat/robot_strat_base.hpp"

namespace goldobot
{

/**  Strat engine (main class)  ***********************************************/

  class RobotStrat : public GoldoThread
  {
  public:
    static RobotStrat& instance();

    RobotStrat();

    int init(char *strat_file_name);

    int read_yaml_conf (char *strat_file_name);

    virtual void taskFunction();

    void start_match();

    void dbg_pause_match();

    void dbg_resume_match();

    void dbg_astar_test(int x_start_mm, int y_start_mm,
                        int x_end_mm, int y_end_mm,
                        int xo1_mm, int yo1_mm,
                        int xo2_mm, int yo2_mm,
                        int xo3_mm, int yo3_mm,
                        char *dump_fname);

    void dbg_dump();

    /* FIXME : TODO : other public methods */

  private:
    int cmd_traj (strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel);

    int cmd_point_to (strat_way_point_t *_wp, float speed, float accel, float deccel);

    int cmd_set_pose (float x_mm, float y_mm, float theta_deg);

    int cmd_nucleo_seq (unsigned int seq_id);

    int cmd_clear_prop_err ();

    /* FIXME : TODO : other private methods */

    char m_strat_file_name[40];

    unsigned char m_nucleo_cmd_buf[1024];

    bool m_start_match_sig;

    /* FIXME : TODO : other private fields */


    static RobotStrat s_instance;
  };
}

