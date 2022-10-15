#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "astar/astar.h"
#include "goldo_thread.hpp"
#include "strat/robot_strat_types.hpp"
#include "strat/robot_strat_base.hpp"
#include "world_state.hpp"

namespace goldobot
{

  /**  Strat SM states  **/
  enum strat_state_t {
    /* global states */
    STRAT_STATE_NULL = 0,
    STRAT_STATE_INIT = 1,
    STRAT_STATE_IDDLE,
    STRAT_STATE_ERROR,
    STRAT_STATE_CANCEL_ERROR,

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

    /* end of match */
    STRAT_STATE_END_MATCH = 128,

    /* debug states */
    STRAT_STATE_PAUSE_DBG = 65536,
    STRAT_STATE_PAUSE2_DBG,
  };

/* FIXME : DEBUG : HACK CRIDF2021 + */
  class TaskCRIDF2021;
/* FIXME : DEBUG : HACK CRIDF2021 - */

/**  Strat engine (main class)  ***********************************************/

  class RobotStrat : public GoldoThread
  {
  public:
    static RobotStrat& instance();

    RobotStrat();

    int init(char *strat_file_name);

    void set_debug(unsigned int debug_flags);

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

  public: /* FIXME : DEBUG : HACK CRDIF2021 (crade!) */
    int cmd_traj (strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel);

    int cmd_point_to (strat_way_point_t *_wp, float speed, float accel, float deccel);

    int cmd_set_pose (float x_mm, float y_mm, float theta_deg);

    int cmd_nucleo_seq (unsigned int seq_id);

    int cmd_clear_prop_err ();

    int cmd_emergency_stop ();

    int cmd_motors_disable ();

    int cmd_propulsion_disable();

  private: /* FIXME : DEBUG : HACK CRDIF2021 (crade!) */
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

    bool m_game_over_flag;

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
    bool m_dbg_no_time_limit;

    /* FIXME : DEBUG */
    StratTask *m_task_dbg;

    /* FIXME : DEBUG */
    char m_dbg_fname[128];

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    TaskCRIDF2021* m_task_cridf2021;
#endif

    static RobotStrat s_instance;
  };

/* FIXME : DEBUG : HACK CRIDF2021 + */
  /**  Strat SM states  **/
  enum task_state_cridf2021_t {
    TASK_STATE_NULL = 0,
    TASK_STATE_IDDLE,

    TASK_STATE_GO_TO_OBSERVATION_POINT,
    TASK_STATE_POINT_TO_PLAYGROUND_CENTER,
    TASK_STATE_PREP_GET_TARGET,
    TASK_STATE_GET_TARGET,
    TASK_STATE_POINT_TO_TARGET,
    TASK_STATE_PREP_GO_TO_TARGET,
    TASK_STATE_GO_TO_TARGET,
    TASK_STATE_CATCH_TARGET,
    TASK_STATE_POINT_TO_HARBOR,
    TASK_STATE_GO_TO_HARBOR,
    TASK_STATE_PREP_ENTER_HARBOR,
    TASK_STATE_ENTER_HARBOR,
    TASK_STATE_EXIT_HARBOR,

    TASK_STATE_EMERGENCY_STOP,
    TASK_STATE_EMERGENCY_WAIT,
    TASK_STATE_EMERGENCY_MOVE_AWAY,
    TASK_STATE_EMERGENCY_ESCAPE,

    /* debug states */
    TASK_STATE_PAUSE_DBG = 128,
  };

  class TaskCRIDF2021
  {
  public:
    bool m_side_is_neg;
    task_state_cridf2021_t m_task_state;
    unsigned int m_act_start_time_ms;
    detected_object_info_t m_target;
    strat_way_point_t m_harbor;
    strat_way_point_t m_obs_pt;
    strat_way_point_t m_obs_dir;
    int m_red_cnt;
    int m_green_cnt;
    bool m_state_change;
    unsigned int m_soft_deadline_ms;
    unsigned int m_hard_deadline_ms;
    unsigned char m_action_buf[4096];

    TaskCRIDF2021(){};
    void init(bool is_blue);
    void set_state(task_state_cridf2021_t new_state, unsigned int _time_ms);
    bool state_exit_test(unsigned int _time_ms,
                         unsigned int soft_deadline_ms,
                         unsigned int hard_deadline_ms);
    void check_deadlines_and_change_state(unsigned int _time_ms,
                                          task_state_cridf2021_t new_state);
    void do_step(float _time_ms);
    strat_action_traj_t * prepare_action_go_to(strat_way_point_t *_target);
    strat_action_point_to_t * prepare_action_point_to(strat_way_point_t *_target);
  };
/* FIXME : DEBUG : HACK CRIDF2021 - */

}

