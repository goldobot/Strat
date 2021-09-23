#pragma once

namespace goldobot
{

/**  Waypoints & position  ****************************************************/

  typedef struct _strat_way_point {
    float x_mm;
    float y_mm;
  } strat_way_point_t;


/**  Strat actions  ***********************************************************/

  typedef enum _strat_action_type {
    STRAT_ACTION_TYPE_NONE = 0,
    STRAT_ACTION_TYPE_WAIT,
    STRAT_ACTION_TYPE_STOP,
    STRAT_ACTION_TYPE_TRAJ,
    STRAT_ACTION_TYPE_POINT_TO,

    STRAT_ACTION_TYPE_NUCLEO_SEQ = 16,

    STRAT_ACTION_TYPE_GOTO_ASTAR = 32,

    STRAT_ACTION_TYPE_BRANCH = 64,

    /* FIXME : TODO : add other types */

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    STRAT_ACTION_TYPE_CRIDF2021 = 128,
#endif
  } strat_action_type_t;

  typedef struct _strat_action_header {
    strat_action_type_t type;
    char label[32];
    int min_duration_ms;
    int max_duration_ms;
  } strat_action_header_t;

  typedef struct _strat_action {
    strat_action_header_t h;
  } strat_action_t;

  typedef struct _strat_action_wait {
    strat_action_header_t h;
  } strat_action_wait_t;

  typedef struct _strat_action_stop {
    strat_action_header_t h;
  } strat_action_stop_t;

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

  typedef struct _strat_action_goto_astar {
    strat_action_header_t h;
    float speed;
    float accel;
    float deccel;
    strat_way_point_t target;
    int nwp;
    strat_way_point_t wp[32];
  } strat_action_goto_astar_t;

  typedef struct _strat_action_branch {
    strat_action_header_t h;
    char condition[64];
    char target_if_true[32];
  } strat_action_branch_t;

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  typedef struct _strat_action_cridf2021 {
    strat_action_header_t h;
  } strat_action_cridf2021_t;
#endif


/**  Strat resources  *********************************************************/

  typedef struct _strat_resource_type {
    char name[64];
    int item_count;
    int item_value;
    int total_value;
  } strat_resource_type_t;

}

