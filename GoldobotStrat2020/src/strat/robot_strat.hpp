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

  /**  Strat SM states  **/
  enum strat_state_t {
    /* global states */
    STRAT_STATE_NULL = 0,
    STRAT_STATE_INIT = 1,
    STRAT_STATE_IDDLE,

    /* task management (complex tasks) */
    STRAT_STATE_SCHEDULE_TASK = 16,
    STRAT_STATE_WAIT_FOR_TASK,
#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    STRAT_STATE_EXEC_TASK_CRIDF2021,
#endif

    /* elementary actions (simple actions) */
    STRAT_STATE_GET_ACTION = 32,
    STRAT_STATE_INIT_ACTION,
    STRAT_STATE_WAIT_END_INIT,
    STRAT_STATE_EXEC_ACTION,
    STRAT_STATE_WAIT_END_ACTION,
    STRAT_STATE_END_ACTION,

    /* emergency states */
    STRAT_STATE_EMERGENCY_STOP = 96,
    STRAT_STATE_EMERGENCY_WAIT,
    STRAT_STATE_EMERGENCY_MOVE_AWAY,
    STRAT_STATE_EMERGENCY_ESCAPE_INIT,
    STRAT_STATE_EMERGENCY_ESCAPE,

    /* debug states */
    STRAT_STATE_PAUSE_DBG = 128,
    STRAT_STATE_PAUSE2_DBG,
  };

/* FIXME : DEBUG : HACK CRIDF2021 + */
  class TaskCRIDF2021
  {
  public:
    strat_state_t m_task_state;

    TaskCRIDF2021(){};
    void init();
    void do_step(float _time);
  };
/* FIXME : DEBUG : HACK CRIDF2021 - */

/**  Strat engine (main class)  ***********************************************/

  class RobotStrat : public GoldoThread
  {
  public:
    static RobotStrat& instance();

    RobotStrat();

    int init(char *strat_file_name);

    void set_debug(bool debug_flag);

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

  private:

    bool check_deadlines_and_change_state(unsigned int my_time_ms,
                                          unsigned int soft_deadline_ms,
                                          unsigned int hard_deadline_ms,
                                          strat_state_t new_strat_state,
                                          const char *done_log);

    bool do_STRAT_STATE_INIT_ACTION(strat_action_t *_act, bool send_prep_cmd);

    void do_STRAT_STATE_EXEC_ACTION(strat_action_t *_act);

    bool is_point_forward(float _x_mm, float _y_mm);

    detected_robot_info_t* get_nearest_obst(int _area_code);

    bool adversary_moved_away();

    strat_action_t * prepare_STRAT_STATE_EMERGENCY_MOVE_AWAY();

    strat_action_t * prepare_STRAT_STATE_EMERGENCY_ESCAPE();

    strat_action_t * prepare_STRAT_STATE_EMERGENCY_RECOVER(strat_action_t * orig_act);

    int cmd_traj (strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel);

    int cmd_point_to (strat_way_point_t *_wp, float speed, float accel, float deccel);

    int cmd_set_pose (float x_mm, float y_mm, float theta_deg);

    int cmd_nucleo_seq (unsigned int seq_id);

    int cmd_clear_prop_err ();

    int cmd_emergency_stop ();

    int cmd_motors_disable ();

    int cmd_propulsion_disable();

    char m_strat_file_name[40];

    int m_n_tasks;

    StratTask *m_task_list[128];

    int m_n_res_types;

    strat_resource_type_t m_res_grabbed[16];

    unsigned char m_nucleo_cmd_buf[1024];

    strat_state_t m_strat_state;

    int m_current_task_idx;

    bool m_start_match_sig;

    bool m_end_match_flag;

    StratPlayground m_path_find_pg;

    AStar m_core_astar;

    bool m_last_move_forwards;

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

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    TaskCRIDF2021 m_task_cridf2021;
#endif

    static RobotStrat s_instance;
  };

}

