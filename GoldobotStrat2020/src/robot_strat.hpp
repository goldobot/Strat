#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_thread.hpp"

namespace goldobot
{

/**  Strat states  ************************************************************/

  typedef enum _strat_state {
    STRAT_STATE_INIT = 0,
    STRAT_STATE_SCHEDULE_TASK,
    STRAT_STATE_ENTER_TASK, /* include path computing */
    STRAT_STATE_GOTO_INIT_POS_TASK,
    STRAT_STATE_INIT_ACTION_TASK,
    STRAT_STATE_EXEC_ACTION_TASK,
    STRAT_STATE_EXIT_TASK,
    STRAT_STATE_IDDLE,
    STRAT_STATE_EMERGENCY_STOP,

    /* FIXME : TODO */

    STRAT_STATE_PAUSE_DBG = 128,
    STRAT_STATE_GET_ACTION_DBG,
    STRAT_STATE_INIT_ACTION_DBG,
    STRAT_STATE_EXEC_ACTION_DBG,
    STRAT_STATE_END_ACTION_DBG,
  } strat_state_t;


/**  Waypoints & position  ****************************************************/

  typedef struct _strat_way_point {
    float x_mm;
    float y_mm;
  } strat_way_point_t;


/**  Strat actions  ***********************************************************/

  typedef enum _strat_action_type {
    STRAT_ACTION_TYPE_NONE = 0,
    STRAT_ACTION_TYPE_WAIT,
    STRAT_ACTION_TYPE_TRAJ,
    STRAT_ACTION_TYPE_POINT_TO,

    /* FIXME : TODO */

    STRAT_ACTION_TYPE_NUCLEO_SEQ = 16,

    STRAT_ACTION_TYPE_GOTO_DBG = 32,
  } strat_action_type_t;

  typedef struct _strat_action_header {
    strat_action_type_t type;
    int min_duration_ms;
    int max_duration_ms;
  } strat_action_header_t;

  typedef struct _strat_action {
    strat_action_header_t h;
  } strat_action_t;

  typedef struct _strat_action_wait {
    strat_action_header_t h;
  } strat_action_wait_t;

  typedef struct _strat_action_traj {
    strat_action_header_t h;
    float speed;
    float accel;
    float deccel;
    int nwp;
    strat_way_point_t wp[32];
  } strat_action_traj_t;

  typedef struct _strat_action_point_to {
    strat_action_header_t h;
    float speed;
    float accel;
    float deccel;
    strat_way_point_t target;
  } strat_action_point_to_t;

  typedef struct _strat_action_nucleo_seq {
    strat_action_header_t h;
    unsigned int nucleo_seq_id;
  } strat_action_nucleo_seq_t;

  typedef struct _strat_action_goto_dbg {
    strat_action_header_t h;
    float speed;
    float accel;
    float deccel;
    strat_way_point_t target;
    int nwp;
    strat_way_point_t wp[32];
  } strat_action_goto_dbg_t;


/**  Strat resources  *********************************************************/

  typedef struct _strat_resource_type {
    char name[64];
    int item_count;
    int item_value;
    int total_value;
  } strat_resource_type_t;


/**  Strat tasks  *************************************************************/

  class StratTask {
  public:
    StratTask();

    /* FIXME : TODO */

    char m_task_name[64];
    int m_curr_act_idx;
    int m_n_actions;
    strat_action_t *m_action_list[128];
    unsigned char m_action_buf[16384];
    int m_priority;
    bool m_started;
    bool m_completed;
    strat_way_point_t m_init_pos_wp;
    strat_way_point_t m_init_point_to_wp;
    int m_min_init_goto_duration_ms;
    int m_max_init_goto_duration_ms;
    float m_required_pos_accuracy_mm;
    float m_required_ang_accuracy_cos;
  };


/**  Strat engine (main class)  ***********************************************/

  class RobotStrat : public GoldoThread
  {
  public:
    static RobotStrat& instance();

    RobotStrat();

    int init(char *strat_file_name);

    virtual void taskFunction();

    void start_match();

    void dbg_pause_match();

    void dbg_resume_match();

    /* FIXME : TODO */

  private:
    int cmd_traj (strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel);

    int cmd_point_to (strat_way_point_t *_wp, float speed, float accel, float deccel);

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

    /* FIXME : DEBUG */
    bool m_dbg_step_by_step;

    /* FIXME : DEBUG */
    bool m_dbg_pause_match_sig;

    /* FIXME : DEBUG */
    bool m_dbg_resume_match_sig;

    /* FIXME : DEBUG */
    StratTask m_task_dbg;

    static RobotStrat s_instance;
  };
}

