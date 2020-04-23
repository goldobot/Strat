#include <string.h>
#include <unistd.h>
#include <math.h>

#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "lidar_detect.hpp"
#include "robot_strat.hpp"


using namespace goldobot;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define MAX(a,b) ((a>b)?a:b)


strat_way_point_t dbg_traj0[] = {
  {  556.0, -1133.0},
  {  336.0, - 990.0},
  {  233.0, - 950.0},
  {  223.0, - 726.0},
  {  626.0, - 396.0},
  { 1013.0, - 470.0},
  { 1280.0, - 096.0},
  { 1153.0,   336.0},
  { 1300.0,   440.0},
  { 1490.0,   300.0},
  { 1740.0,   290.0},
};

strat_way_point_t dbg_point_to_point0 = {1490.0, 300.0};

strat_way_point_t dbg_target_point1 = {1850.0, -1350.0};

strat_way_point_t dbg_point_to_point1 = {1850.0, -790.0};

strat_way_point_t dbg_traj2[] = {
  //{ 1850.0, -1000.0},
  { 1850.0,  -790.0},
};

strat_way_point_t dbg_point_to_point2 = {1850.0, -790.0};

strat_way_point_t dbg_target_point3 = {300.0, -790.0};

strat_way_point_t dbg_point_to_point3 = {300.0, -1350.0};

strat_way_point_t dbg_target_point4 = {300.0, -1350.0};

strat_way_point_t dbg_point_to_point4 = {300.0, -790.0};


/******************************************************************************/
/**  StratTask  ***************************************************************/
/******************************************************************************/

StratTask::StratTask()
{
  memset (m_task_name, 0, sizeof(m_task_name));
  m_n_actions = 0;
  m_curr_act_idx = 0;
  memset (m_action_list, 0, sizeof(m_action_list));
  memset (m_action_buf, 0, sizeof(m_action_buf));
  m_priority = 0;
  m_started = false;
  m_completed = false;
  memset (&m_init_pos_wp, 0, sizeof(m_init_pos_wp));
  memset (&m_init_point_to_wp, 0, sizeof(m_init_point_to_wp));
  m_min_init_goto_duration_ms = 100.0;
  m_max_init_goto_duration_ms = 20000.0; /* FIXME : TODO : heuristics.. */
  m_required_pos_accuracy_mm = 50.0;
  m_required_ang_accuracy_cos = 0.999; /* WARNING : NOT RADIANS! */
}


/******************************************************************************/
/**  RobotStrat  **************************************************************/
/******************************************************************************/

RobotStrat RobotStrat::s_instance;

RobotStrat& RobotStrat::instance()
{
  return s_instance;
}

RobotStrat::RobotStrat()
{
  strncpy(m_thread_name,"RobotStrat",sizeof(m_thread_name));

  memset (m_strat_file_name, 0, sizeof(m_strat_file_name));

  m_n_tasks = 0;

  memset (m_task_list, 0, sizeof(m_task_list));

  m_n_res_types = 0;

  memset (m_res_grabbed, 0, sizeof(m_res_grabbed));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  m_strat_state = STRAT_STATE_INIT;

  m_emerg_int_state = STRAT_STATE_INIT;

  m_current_task_idx = 0;

  m_start_match_sig = false;

  m_dbg_step_by_step = false;

  m_dbg_pause_match_sig = false;

  m_dbg_resume_match_sig = false;

  /* FIXME : TODO */

}

int RobotStrat::init(char *strat_file_name)
{
  strncpy(m_strat_file_name, strat_file_name, sizeof(m_strat_file_name));

  m_n_tasks = 0;

  memset (m_task_list, 0, sizeof(m_task_list));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  m_n_res_types = 0;

  memset (m_res_grabbed, 0, sizeof(m_res_grabbed));

  m_strat_state = STRAT_STATE_INIT;

  m_emerg_int_state = STRAT_STATE_INIT;

  m_current_task_idx = 0;

  m_start_match_sig = false;

  m_dbg_step_by_step = false;

  m_dbg_pause_match_sig = false;

  m_dbg_resume_match_sig = false;

  /* FIXME : TODO */

  return 0;
}

void RobotStrat::taskFunction()
{

  m_task_running = true;

  /* FIXME : TODO : synchronise with comm threads to avoid fake event 
     detection */
  usleep (100000);

  /* FIXME : DEBUG */
  m_dbg_step_by_step = true;

  while(!m_stop_task)
  {
    struct timespec my_tp;
    unsigned int my_time_ms = 0;
    unsigned int soft_deadline_ms = 0;
    unsigned int hard_deadline_ms = 0;
    bool state_change_dbg = true;
    strat_action_t *my_action = NULL;
    bool action_ok = false;

    volatile unsigned int my_robot_sensors = 
      RobotState::instance().m_robot_sensors;

    clock_gettime(1, &my_tp);
    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

    switch (m_strat_state) {
    case STRAT_STATE_INIT:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_INIT *********************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      /* FIXME : DEBUG */
      m_task_dbg.m_curr_act_idx = 0;

      if (((my_robot_sensors&2)==0))
      {
        printf ("\n DEBUG : Tirette!..\n\n");

        /* FIXME : TODO : necessary? */
        usleep (3000000);

        m_start_match_sig = true;

        m_strat_state = STRAT_STATE_GET_ACTION_DBG;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_GET_ACTION_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_GET_ACTION_DBG ***********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (m_task_dbg.m_curr_act_idx < m_task_dbg.m_n_actions)
      {
        my_action = m_task_dbg.m_action_list[m_task_dbg.m_curr_act_idx];

        if (my_action==NULL)
        {
          printf (" Warning : NULL action pointer!\n");
          printf ("\n");
          m_strat_state = STRAT_STATE_END_ACTION_DBG;
          state_change_dbg = true;
        }
        else
        {
          printf (" Got action : %d.\n", my_action->h.type);
          printf ("\n");
          m_strat_state = STRAT_STATE_INIT_ACTION_DBG;
          state_change_dbg = true;
        }
      }
      else
      {
        m_strat_state = STRAT_STATE_IDDLE;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_INIT_ACTION_DBG:
      action_ok = false;

      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_INIT_ACTION_DBG **********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      soft_deadline_ms = my_time_ms + my_action->h.min_duration_ms;
      hard_deadline_ms = my_time_ms + my_action->h.max_duration_ms;

      switch (my_action->h.type) {
      case STRAT_ACTION_TYPE_WAIT:
        printf (" STRAT_ACTION_TYPE_WAIT\n");
        action_ok = true;
        break;
      case STRAT_ACTION_TYPE_TRAJ:
        printf (" STRAT_ACTION_TYPE_TRAJ\n");
        action_ok = true;
        /* FIXME : TODO */
        //cmd_traj (dbg_traj0, _countof(dbg_traj0), 0.4, 0.3, 0.3);
        break;
      case STRAT_ACTION_TYPE_POINT_TO:
        printf (" STRAT_ACTION_TYPE_POINT_TO\n");
        action_ok = true;
        /* FIXME : TODO */
        break;
      case STRAT_ACTION_TYPE_NUCLEO_SEQ:
        printf (" STRAT_ACTION_TYPE_NUCLEO_SEQ\n");
        action_ok = true;
        /* FIXME : TODO */
        break;
      case STRAT_ACTION_TYPE_GOTO_DBG:
        printf (" STRAT_ACTION_TYPE_GOTO_DBG\n");
        action_ok = true;
        /* FIXME : TODO */
        break;
      default:
        printf (" Warning : Unknown action type!\n");
        action_ok = false;
      }

      if (action_ok)
      {
        m_strat_state = STRAT_STATE_EXEC_ACTION_DBG;
        state_change_dbg = true;
      }
      else
      {
        m_strat_state = STRAT_STATE_END_ACTION_DBG;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_EXEC_ACTION_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EXEC_ACTION_DBG **********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (my_time_ms > soft_deadline_ms)
      {
        if (my_time_ms < (soft_deadline_ms+20)) printf (".");
        /* FIXME : TODO : add test for physical action completion */
      }

      if (my_time_ms > hard_deadline_ms)
      {
        printf ("\n");
        m_strat_state = STRAT_STATE_END_ACTION_DBG;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_END_ACTION_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_END_ACTION_DBG ***********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      m_task_dbg.m_curr_act_idx++;

      if (m_task_dbg.m_curr_act_idx < m_task_dbg.m_n_actions)
      {
        if (m_dbg_pause_match_sig)
        {
          m_dbg_pause_match_sig = false;
          m_strat_state = STRAT_STATE_PAUSE_DBG;
        }
        else if (m_dbg_step_by_step)
        {
          m_strat_state = STRAT_STATE_PAUSE_DBG;
        }
        else
        {
          m_strat_state = STRAT_STATE_GET_ACTION_DBG;
        }
        state_change_dbg = true;
      }
      else
      {
        m_strat_state = STRAT_STATE_IDDLE;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_PAUSE_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_PAUSE_DBG ****************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (m_dbg_resume_match_sig)
      {
        m_dbg_resume_match_sig = false;
        m_strat_state = STRAT_STATE_GET_ACTION_DBG;
        state_change_dbg = true;
      }
      break;

    case STRAT_STATE_IDDLE:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_IDDLE ********************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      /* FIXME : TODO */
      break;

    case STRAT_STATE_EMERGENCY_STOP:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EMERGENCY_STOP ***********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      /* FIXME : TODO */
      break;

    default:
      printf ("\n");
      printf ("****************************************\n");
      printf ("* Warning : unknown STRAT state! (%d)\n",m_strat_state);
      printf ("****************************************\n");
      printf ("\n");
      m_strat_state = STRAT_STATE_IDDLE;
      state_change_dbg = true;
    }


    usleep (10000);

    pthread_yield();
  } /* while(!m_stop_task) */

  m_task_running = false;
}

void RobotStrat::start_match()
{
  if (!m_task_running) return;

  m_start_match_sig = true;
}

void RobotStrat::dbg_pause_match()
{
  if (!m_task_running) return;

  m_dbg_pause_match_sig = true;
}

void RobotStrat::dbg_resume_match()
{
  if (!m_task_running) return;

  m_dbg_resume_match_sig = true;
}

int RobotStrat::cmd_traj(strat_way_point_t *_wp, int _nwp, float speed, float accel, float deccel)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0055; /*DbgPropulsionExecuteTrajectory*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&speed, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&accel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&deccel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  for (int i=0; i<_nwp; i++) 
  {
    unsigned int my_x = _wp[i].x_mm*0.001;
    unsigned int my_y = _wp[i].y_mm*0.001;

    field_len = sizeof(float);
    memcpy (_pc, (unsigned char *)&(my_x), field_len);
    _pc += field_len;
    cmd_buf_len += field_len;

    field_len = sizeof(float);
    memcpy (_pc, (unsigned char *)&(my_y), field_len);
    _pc += field_len;
    cmd_buf_len += field_len;
  }

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_point_to(strat_way_point_t *_wp, float speed, float accel, float deccel)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0058; /*DbgPropulsionExecutePointTo*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  unsigned int my_x = _wp->x_mm*0.001;
  unsigned int my_y = _wp->y_mm*0.001;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&(my_x), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&(my_y), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&speed, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&accel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(float);
  memcpy (_pc, (unsigned char *)&deccel, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  return 0;
}


