#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "astar/astar.h"
#include "goldo_thread.hpp"
#include "strat/robot_strat_types.hpp"
#include "strat/robot_strat_base.hpp"

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

    /* FIXME : TODO */

    /**  Strat SM states  **/
    enum strat_state_t {
      STRAT_STATE_INIT = 0,
      STRAT_STATE_SCHEDULE_TASK,
      STRAT_STATE_ENTER_TASK, /* include path computing */
      STRAT_STATE_GOTO_INIT_POS_TASK,
      STRAT_STATE_INIT_ACTION_TASK,
      STRAT_STATE_EXEC_ACTION_TASK,
      STRAT_STATE_WAIT_END_ACTION,
      STRAT_STATE_END_ACTION,
      STRAT_STATE_EXIT_TASK,
      STRAT_STATE_IDDLE,
      STRAT_STATE_EMERGENCY_STOP,

      /* FIXME : TODO */

      STRAT_STATE_PAUSE_DBG = 128,
      STRAT_STATE_PAUSE2_DBG,
      STRAT_STATE_GET_ACTION_DBG,
      STRAT_STATE_INIT_ACTION_DBG,
      STRAT_STATE_WAIT_END_INIT_DBG,
      STRAT_STATE_EXEC_ACTION_DBG,
      STRAT_STATE_WAIT_END_ACTION_DBG,
      STRAT_STATE_END_ACTION_DBG,
    };

  private:
    int cmd_traj (strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel);

    int cmd_point_to (strat_way_point_t *_wp, float speed, float accel, float deccel);

    int cmd_set_pose (float x_mm, float y_mm, float theta_deg);

    int cmd_nucleo_seq (unsigned int seq_id);

    int cmd_clear_prop_err ();

    /* FIXME : TODO */

    char m_strat_file_name[40];

    int m_n_tasks;

    StratTask *m_task_list[128];

    int m_n_res_types;

    strat_resource_type_t m_res_grabbed[16];

    unsigned char m_nucleo_cmd_buf[1024];

    strat_state_t m_strat_state;

    strat_state_t m_emerg_int_state;

    int m_current_task_idx;

    bool m_start_match_sig;

    StratPlayground m_path_find_pg;

    AStar m_core_astar;


    /* FIXME : DEBUG */
    bool m_dbg_step_by_step;

    /* FIXME : DEBUG */
    bool m_dbg_pause_match_sig;

    /* FIXME : DEBUG */
    bool m_dbg_resume_match_sig;

    /* FIXME : DEBUG */
    StratTask *m_task_dbg;

    /* FIXME : DEBUG */
    char m_dbg_fname[128];


    static RobotStrat s_instance;
  };
}

