#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#include <string.h>
#include <unistd.h>
#include <math.h>
#include <zmq.h>

#include <fstream>
#include <iostream>

#include "goldo_conf.hpp"
#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "world_state.hpp"
#include "detect/lidar_detect.hpp"
#include "strat/robot_strat_types.hpp"
#include "strat/robot_strat_base.hpp"
#include "strat/robot_strat.hpp"

/* imported from Nucleo project (Carte_GR_SW4STM32) */
#include "message_types.hpp"

using namespace goldobot;

/* FIXME : DEBUG */
#define DEBUG_OBST_AVOIDANCE 1

#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
int s_moved_away_reason = 0;
float s_x_mm = 0.0;
float s_y_mm = 0.0;
float s_theta_rad = 0.0;
float s_obst_x_mm = 0.0;
float s_obst_y_mm = 0.0;
float s_delta_x_mm = 0.0;
float s_delta_y_mm = 0.0;
#endif

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define MAX(a,b) ((a>b)?a:b)


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
  strncpy(m_thread_name,"RobotStrat",sizeof(m_thread_name)-1);

  memset (m_strat_file_name, 0, sizeof(m_strat_file_name));

  m_n_tasks = 0;

  memset (m_task_list, 0, sizeof(m_task_list));

  m_n_res_types = 0;

  memset (m_res_grabbed, 0, sizeof(m_res_grabbed));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  m_strat_state = STRAT_STATE_INIT;

  m_current_task_idx = 0;

  m_start_match_sig = false;

  m_end_match_flag = false;

  //m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);

  m_last_move_forwards = true;


  /* DEBUG */
  m_task_dbg = new StratTask();

  m_dbg_step_by_step = false;

  m_dbg_pause_match_sig = false;

  m_dbg_resume_match_sig = false;

  memset (m_dbg_fname, 0, sizeof(m_dbg_fname));

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  m_task_cridf2021 = new TaskCRIDF2021();
#endif
}

int RobotStrat::init(char *strat_file_name)
{
  strncpy(m_strat_file_name, strat_file_name, sizeof(m_strat_file_name)-1);

  m_n_tasks = 0;

  memset (m_task_list, 0, sizeof(m_task_list));

  memset (m_nucleo_cmd_buf, 0, sizeof(m_nucleo_cmd_buf));

  m_n_res_types = 0;

  memset (m_res_grabbed, 0, sizeof(m_res_grabbed));

  m_strat_state = STRAT_STATE_INIT;

  m_current_task_idx = 0;

  m_start_match_sig = false;

  m_end_match_flag = false;

  m_path_find_pg.init();

  m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);

#if 0/* FIXME : DEBUG : EXPERIMENTAL */
  read_yaml_conf (m_strat_file_name);
#endif

  m_last_move_forwards = true;


  /* DEBUG */
  m_dbg_step_by_step = false;

  m_dbg_pause_match_sig = false;

  m_dbg_resume_match_sig = false;

  memset (m_dbg_fname, 0, sizeof(m_dbg_fname));

  return 0;
}

void RobotStrat::set_debug(bool debug_flag)
{
  m_dbg_step_by_step = debug_flag;
}

int RobotStrat::read_yaml_conf(char *fname)
{
  std::ifstream fin;
  YAML::Node yconf;
  YAML::Node dbg_node;

  try 
  {
    fin.open(fname);
    yconf = YAML::Load(fin);
  } 
  catch(const YAML::Exception& e)
  {
    printf ("  ERROR : %s\n", e.what());
    return -1;
  }

  /* Load debug task */
  dbg_node = yconf["dbg_task"];
  if (!dbg_node) 
  {
    printf ("  ERROR : no 'dbg_task' section\n");
    return -1;
  }
  if (m_task_dbg->read_yaml_conf(dbg_node)!=0)
  {
    return -1;
  }

  /* FIXME : TODO */

  return 0;
}

void RobotStrat::taskFunction()
{
  bool state_change_dbg = true;
  unsigned int soft_deadline_ms = 0;
  unsigned int hard_deadline_ms = 0;
  unsigned int recov_fail_cnt = 0;
  strat_action_t *my_action = NULL;
  strat_action_t *my_escape_action = NULL;
  unsigned int match_start_ms = 0;
  bool match_funny_done = false;

  m_task_running = true;

  /* FIXME : TODO : synchronise with comm threads to avoid fake event 
     detection */
#ifndef WIN32
  usleep(100000);
#else
  Sleep(100);
#endif

  /* FIXME : DEBUG */
  //m_dbg_step_by_step = true;

  int log_lidar_fd;
  char log_lidar_write_buf[64];
  /* FIXME : TODO : refactor */
  log_lidar_fd = open ("/home/pi/goldo/strat2020/log/log_lidar.txt", O_RDWR);


#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
  goldo_conf_info_t& ci = GoldoConf::instance().c();
  unsigned int robot_sensors = RobotState::instance().s().robot_sensors;
  bool is_blue = ((robot_sensors&RobotState::GPIO_SIDE_SELECT_MASK) == 0);
  if (robot_sensors&0x00000020)
  {
    printf (" DEBUG: HACK: set 2 obstacles\n");
    LidarDetect::instance().set_nb_of_send_detect(2);
  }
  else
  {
    printf (" DEBUG: HACK: set 1 obstacle\n");
    LidarDetect::instance().set_nb_of_send_detect(1);
  }
  if (is_blue) /* Y - */
  {
    if (ci.conf_strat_file_neg_str[0]==0x00)
    {
      read_yaml_conf (m_strat_file_name);
    }
    else
    {
      read_yaml_conf (ci.conf_strat_file_neg_str);
    }
  }
  else /* Y + */
  {
    if (ci.conf_strat_file_pos_str[0]==0x00)
    {
      read_yaml_conf (m_strat_file_name);
    }
    else
    {
      read_yaml_conf (ci.conf_strat_file_pos_str);
    }
  }
  printf (" DEBUG: robot_sensors: %.8x\n", robot_sensors);
  printf (" DEBUG: color: %s\n", is_blue?"BLUE":"YELLOW");
  printf (" DEBUG: task_name: %s\n", m_task_dbg->m_task_name);
#endif

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  m_task_cridf2021->init(is_blue);
#endif

  while(!m_stop_task)
  {
    struct timespec my_tp;
    unsigned int my_time_ms = 0;
    bool action_ok = false;

    clock_gettime(1, &my_tp);
    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

    if (!m_dbg_step_by_step)
    {
      if ((!match_funny_done) && (match_start_ms!=0) && (my_time_ms>(match_start_ms+97000)))
      {
        printf ("\n");
        printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf ("!!!!!!!!!!!!  FUNNY ACTION  !!!!!!!!!!!!\n");
        printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf ("\n");
        printf ("match_time = %8.3f\n", 0.001*(my_time_ms-match_start_ms));
        printf ("\n");
#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
        RobotState::instance().m_lidar_detection_enabled = false;
        RobotState::instance().set_obstacle_gpio(false);
#endif
        cmd_nucleo_seq (9);
        match_funny_done = true;
      }

      if ((match_start_ms!=0) && (my_time_ms>(match_start_ms+100000)) && (!m_end_match_flag))
      {
        m_end_match_flag = true;
        RobotState::instance().s().strat_stop = true;
        cmd_emergency_stop();
        cmd_propulsion_disable();
        cmd_motors_disable();
        m_strat_state = STRAT_STATE_IDDLE;
        printf ("\n");
        printf ("GAME OVER!\n");
        printf ("match_time = %8.3f\n", 0.001*(my_time_ms-match_start_ms));
        printf ("\n");
        state_change_dbg = true;
      }
    }

    if ((match_start_ms!=0) && (my_time_ms<(match_start_ms+100000)))
    {
      sprintf (log_lidar_write_buf,"%d %d\n",my_time_ms-match_start_ms,RobotState::instance().s().obstacle_plot_cnt);
      write (log_lidar_fd,log_lidar_write_buf,strlen(log_lidar_write_buf));
    }

    if (RobotState::instance().emergency_stop() && 
        (m_strat_state!=STRAT_STATE_IDDLE) &&
        (m_strat_state!=STRAT_STATE_EMERGENCY_STOP) &&
        (m_strat_state!=STRAT_STATE_EMERGENCY_WAIT) &&
        (m_strat_state!=STRAT_STATE_EMERGENCY_MOVE_AWAY) )
    {
      printf ("EMERGENCY_STOP!\n");
      printf ("  last move dir : %s\n", m_last_move_forwards?"FORWARDS":"BACKWARDS");
      cmd_clear_prop_err();
      soft_deadline_ms = my_time_ms + 500;
      hard_deadline_ms = my_time_ms + m_task_dbg->m_obstacle_freeze_timeout_ms;
      recov_fail_cnt = 0;
      m_strat_state = STRAT_STATE_EMERGENCY_STOP;
      state_change_dbg = true;
    }

    if ((m_strat_state!=STRAT_STATE_IDDLE) &&
        (m_strat_state!=STRAT_STATE_EMERGENCY_WAIT) &&
        (m_strat_state!=STRAT_STATE_EMERGENCY_STOP) )
    {
      if (RobotState::instance().s().speed_abs>0.01) 
      {
        m_last_move_forwards = RobotState::instance().s().forward_move;
      }
    }

    /*************************************/
    /*************************************/
    /** THE MAIN STRATEGY STATE MACHINE **/
    /*************************************/
    /*************************************/
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

      /* FIXME : TODO : define random entry point for strategy? */
      m_task_dbg->m_curr_act_idx = 0;

      if ((!RobotState::instance().tirette_present()) || m_start_match_sig)
      {
        match_start_ms = my_time_ms;

        printf ("\n");
        printf ("match_time = 0\n");
        printf ("\n");
        printf ("\n DEBUG : GO!..\n\n");

        /* FIXME : TODO : necessary? */
#ifndef WIN32
        usleep(100000);
#else
        Sleep(100);
#endif

        m_strat_state = STRAT_STATE_GET_ACTION;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_GET_ACTION:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_GET_ACTION ***************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (m_task_dbg->m_curr_act_idx < m_task_dbg->m_n_actions)
      {
        my_action = m_task_dbg->m_action_list[m_task_dbg->m_curr_act_idx];

        if ((my_action!=NULL) && (my_action->h.type==STRAT_ACTION_TYPE_BRANCH))
        {
          strat_action_branch_t *act_branch = (strat_action_branch_t *)my_action;
          bool condition = false;
          printf ("\n");
          printf ("@@@@@@@@@@\n");
          printf ("@ BRANCH @\n");
          printf ("@@@@@@@@@@\n");
          printf ("\n");
          printf (" Got conditional BRANCH with target: %s\n", act_branch->target_if_true);
          /* FIXME : TODO : really parse condition string */
          condition = WorldState::instance().get_observable_value(act_branch->condition);
          printf (" Condition '%s' is %s\n", act_branch->condition, condition?"TRUE":"FALSE");
          if (condition)
          {
            int target_idx = m_task_dbg->get_action_idx_with_label(act_branch->target_if_true);
            if (target_idx<0)
            {
              printf (" Warning : Cannot find action with label '%s' !\n", act_branch->target_if_true);
              /* FIXME : TODO : what to do?.. */
              my_action = NULL;
            }
            else
            {
              m_task_dbg->m_curr_act_idx = target_idx;
              my_action = m_task_dbg->m_action_list[m_task_dbg->m_curr_act_idx];
            }
          }
          else
          {
            m_task_dbg->m_curr_act_idx++;
            my_action = m_task_dbg->m_action_list[m_task_dbg->m_curr_act_idx];
          }
        }

        if (my_action==NULL)
        {
          printf (" Warning : NULL action pointer!\n");
          printf ("\n");
          m_strat_state = STRAT_STATE_IDDLE;
          state_change_dbg = true;
        }
        else
        {
          if (my_action->h.type==STRAT_ACTION_TYPE_GOTO_ASTAR)
          {
            strat_action_goto_astar_t *act_ast= 
              (strat_action_goto_astar_t *) my_action;
            m_task_dbg->m_current_action_final_wp = act_ast->target;
          }
          else if (my_action->h.type==STRAT_ACTION_TYPE_TRAJ)
          {
            strat_action_traj_t *act_traj= 
              (strat_action_traj_t *) my_action;
            m_task_dbg->m_current_action_final_wp = act_traj->wp[act_traj->nwp-1];
          }
          else
          {
            m_task_dbg->m_current_action_final_wp.x_mm = 1000.0;
            m_task_dbg->m_current_action_final_wp.y_mm = 0.0;
          }
          printf (" Got action : %d.\n", my_action->h.type);
          printf (" Final wp : <%f,%f>\n",
                  m_task_dbg->m_current_action_final_wp.x_mm,
                  m_task_dbg->m_current_action_final_wp.y_mm);
          printf ("\n");
          m_strat_state = STRAT_STATE_INIT_ACTION;
          state_change_dbg = true;
        }
      }
      else
      {
        m_strat_state = STRAT_STATE_IDDLE;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_INIT_ACTION:
      action_ok = false;

      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_INIT_ACTION **************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      action_ok = do_STRAT_STATE_INIT_ACTION(my_action, true);

      if (my_action->h.type==STRAT_ACTION_TYPE_GOTO_ASTAR)
      {
        soft_deadline_ms = my_time_ms + my_action->h.min_duration_ms;
        hard_deadline_ms = my_time_ms + my_action->h.max_duration_ms;
      }

      if (action_ok)
      {
        if (m_dbg_step_by_step)
        {
          m_strat_state = STRAT_STATE_PAUSE2_DBG;
        }
        else
        {
          m_strat_state = STRAT_STATE_WAIT_END_INIT;
        }
        state_change_dbg = true;
      }
      else
      {
        /* FIXME : TODO : is this OK? */
        //m_strat_state = STRAT_STATE_END_ACTION;
        m_strat_state = STRAT_STATE_IDDLE;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_WAIT_END_INIT:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_WAIT_END_INIT ************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (my_action->h.type==STRAT_ACTION_TYPE_GOTO_ASTAR)
      {
        state_change_dbg = check_deadlines_and_change_state(
          my_time_ms, soft_deadline_ms, hard_deadline_ms, STRAT_STATE_EXEC_ACTION, "INIT DONE");
      }
      else
      {
        printf ("\n");
        printf (" No prep action. SKIPPING init wait state.\n");
        m_strat_state = STRAT_STATE_EXEC_ACTION;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_EXEC_ACTION:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EXEC_ACTION **************\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      soft_deadline_ms = my_time_ms + my_action->h.min_duration_ms;
      hard_deadline_ms = my_time_ms + my_action->h.max_duration_ms;

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
      if (my_action->h.type==STRAT_ACTION_TYPE_CRIDF2021)
      {
        m_strat_state = STRAT_STATE_EXEC_TASK_CRIDF2021;
        state_change_dbg = true;
      }
      else
#endif
      {
        do_STRAT_STATE_EXEC_ACTION(my_action);

        m_strat_state = STRAT_STATE_WAIT_END_ACTION;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_WAIT_END_ACTION:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_WAIT_END_ACTION **********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      state_change_dbg = check_deadlines_and_change_state(
        my_time_ms, soft_deadline_ms, hard_deadline_ms, STRAT_STATE_END_ACTION, "EXEC DONE");

      break;

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    case STRAT_STATE_EXEC_TASK_CRIDF2021:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EXEC_TASK_CRIDF2021 ******\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
        m_task_cridf2021->set_state(TASK_STATE_GO_TO_OBSERVATION_POINT, my_time_ms);
      }

      if (my_time_ms > hard_deadline_ms)
      {
        printf ("\n");
        m_strat_state = STRAT_STATE_END_ACTION;
        state_change_dbg = true;
      }
      else
      {
        m_task_cridf2021->do_step(my_time_ms);
      }

      break;
#endif

    case STRAT_STATE_END_ACTION:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_END_ACTION ***************\n");
        printf ("****************************************\n");
        printf ("\n");
        printf ("match_time = %8.3f\n", 0.001*(my_time_ms-match_start_ms));
        printf ("\n");
        state_change_dbg = false;
      }

      m_task_dbg->m_curr_act_idx++;

      if (m_task_dbg->m_curr_act_idx < m_task_dbg->m_n_actions)
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
          m_strat_state = STRAT_STATE_GET_ACTION;
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
        //printf ("****************************************\n");
        //printf ("* STRAT_STATE_PAUSE_DBG ****************\n");
        //printf ("****************************************\n");
        printf ("! DBG_PAUSE\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (m_dbg_resume_match_sig)
      {
        m_dbg_resume_match_sig = false;
        m_strat_state = STRAT_STATE_GET_ACTION;
        state_change_dbg = true;
      }
      break;

    case STRAT_STATE_PAUSE2_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        //printf ("****************************************\n");
        //printf ("* STRAT_STATE_PAUSE2_DBG ***************\n");
        //printf ("****************************************\n");
        printf ("! DBG_PAUSE\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (m_dbg_resume_match_sig)
      {
        m_dbg_resume_match_sig = false;
        m_strat_state = STRAT_STATE_WAIT_END_INIT;
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
        printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf ("! STRAT_STATE_EMERGENCY_STOP !!!!!!!!!!!\n");
        printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf ("\n");
        state_change_dbg = false;
      }

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
      if (my_action->h.type==STRAT_ACTION_TYPE_CRIDF2021)
      {
        m_strat_state = STRAT_STATE_EXEC_TASK_CRIDF2021;
        m_task_cridf2021->set_state(TASK_STATE_EMERGENCY_WAIT, my_time_ms);
        state_change_dbg = true;
      }
      else
#endif
      {
        m_strat_state = STRAT_STATE_EMERGENCY_WAIT;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_EMERGENCY_WAIT:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EMERGENCY_WAIT ***********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
      s_moved_away_reason = 0;
#endif
      if ((my_time_ms > soft_deadline_ms) && adversary_moved_away())
      {
        printf ("\n");
        printf ("Way cleared, trying to continue last action..\n");
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
        printf ("  s_moved_away_reason = %d\n", s_moved_away_reason);
        printf ("  s_x_mm = %f\n", s_x_mm);
        printf ("  s_y_mm = %f\n", s_y_mm);
        printf ("  s_theta_rad = %f\n", s_theta_rad);
        printf ("  s_obst_x_mm = %f\n", s_obst_x_mm);
        printf ("  s_obst_y_mm = %f\n", s_obst_y_mm);
        printf ("  s_delta_x_mm = %f\n", s_delta_x_mm);
        printf ("  s_delta_y_mm = %f\n", s_delta_y_mm);
#endif
        printf ("\n");

#if 0 /* FIXME : DEBUG */
        {
          printf ("FIXME : DEBUG : forcing IDDLE state\n");
          m_strat_state = STRAT_STATE_IDDLE;
          state_change_dbg = true;
        }
#else
        (void) prepare_STRAT_STATE_EMERGENCY_RECOVER(my_action);
        action_ok = do_STRAT_STATE_INIT_ACTION(my_action, false);
        if (!action_ok)
        {
          printf ("Failed to recover action! (%d tries)\n", recov_fail_cnt);
          recov_fail_cnt++;
          if (recov_fail_cnt>20)
          {
#if 1 /* FIXME : TODO : what to do next?!.. */
            m_strat_state = STRAT_STATE_IDDLE;
            state_change_dbg = true;
#endif
          }
          else
          {
            soft_deadline_ms += 500;
          }
        }
        else
        {
          soft_deadline_ms = my_time_ms + my_action->h.min_duration_ms;
          hard_deadline_ms = my_time_ms + my_action->h.max_duration_ms;
          recov_fail_cnt = 0;
          do_STRAT_STATE_EXEC_ACTION(my_action);
          m_strat_state = STRAT_STATE_WAIT_END_ACTION;
          state_change_dbg = true;
        }
#endif
      }

      if (my_time_ms > hard_deadline_ms)
      {
        printf ("\n");
        printf ("Obstacle still present, trying to move away..\n");
        printf ("\n");

        /* FIXME : TODO : is this necessary? */
        RobotState::instance().m_lidar_detection_enabled = false;
        RobotState::instance().set_obstacle_gpio(false);

        auto move_away_action = prepare_STRAT_STATE_EMERGENCY_MOVE_AWAY();
        do_STRAT_STATE_INIT_ACTION(move_away_action, true);
        do_STRAT_STATE_EXEC_ACTION(move_away_action);
        soft_deadline_ms = my_time_ms + move_away_action->h.min_duration_ms;
        hard_deadline_ms = my_time_ms + move_away_action->h.max_duration_ms;
        m_strat_state = STRAT_STATE_EMERGENCY_MOVE_AWAY;
        state_change_dbg = true;
      }

      break;

    case STRAT_STATE_EMERGENCY_MOVE_AWAY:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EMERGENCY_MOVE_AWAY ******\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      state_change_dbg = check_deadlines_and_change_state(
        my_time_ms, soft_deadline_ms, hard_deadline_ms, STRAT_STATE_NULL, "MOVE_AWAY DONE");
      if (state_change_dbg)
      {
        RobotState::instance().m_lidar_detection_enabled = true;
        my_escape_action = prepare_STRAT_STATE_EMERGENCY_ESCAPE();
        soft_deadline_ms = my_time_ms + my_escape_action->h.min_duration_ms;
        hard_deadline_ms = my_time_ms + my_escape_action->h.max_duration_ms;
        action_ok = do_STRAT_STATE_INIT_ACTION(my_escape_action, true);
        if (action_ok)
        {
          m_strat_state = STRAT_STATE_EMERGENCY_ESCAPE_INIT;
        }
        else
        {
          printf ("Failed to init EMERGENCY_ESCAPE!\n");
#if 1 /* FIXME : TODO : what to do next?!.. */
          m_strat_state = STRAT_STATE_IDDLE;
          state_change_dbg = true;
#endif
        }
      }

      break;

    case STRAT_STATE_EMERGENCY_ESCAPE_INIT:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EMERGENCY_ESCAPE_INIT ****\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      state_change_dbg = check_deadlines_and_change_state(
        my_time_ms, soft_deadline_ms, hard_deadline_ms, STRAT_STATE_EMERGENCY_ESCAPE, "ESCAPE INIT DONE");
      if (state_change_dbg)
      {
        soft_deadline_ms = my_time_ms + my_escape_action->h.min_duration_ms;
        hard_deadline_ms = my_time_ms + my_escape_action->h.max_duration_ms;
        do_STRAT_STATE_EXEC_ACTION(my_escape_action);
      }

      break;

    case STRAT_STATE_EMERGENCY_ESCAPE:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_EMERGENCY_ESCAPE *********\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      state_change_dbg = check_deadlines_and_change_state(
        my_time_ms, soft_deadline_ms, hard_deadline_ms, STRAT_STATE_END_ACTION, "ESCAPE DONE");

      break;

    default:
      printf ("\n");
      printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      printf ("! Warning : unknown STRAT state! (%d)\n",m_strat_state);
      printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      printf ("\n");
      m_strat_state = STRAT_STATE_IDDLE;
      state_change_dbg = true;
    }

#ifndef WIN32
    usleep(10000);
#else
    Sleep(10);
#endif

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  } /* while(!m_stop_task) */

  close (log_lidar_fd);

  m_task_running = false;
}

bool RobotStrat::check_deadlines_and_change_state(unsigned int my_time_ms,
                                                  unsigned int soft_deadline_ms,
                                                  unsigned int hard_deadline_ms,
                                                  strat_state_t new_strat_state,
                                                  const char *done_log)
{
  bool state_changed = false;

  if (my_time_ms > soft_deadline_ms)
  {
    if (my_time_ms < (soft_deadline_ms+20)) printf (".");
    /* FIXME : TODO : add completion test for actuators */
    if (!RobotState::instance().propulsion_busy())
    {
      printf ("\n");
      printf (" %s\n", done_log);
      if (new_strat_state!=STRAT_STATE_NULL)
        m_strat_state = new_strat_state;
      state_changed = true;
    }
    /* FIXME : TODO : error management */
    else if (RobotState::instance().propulsion_error())
    {
      printf ("\n");
      printf (" Propulsion ERROR\n");
      m_strat_state = STRAT_STATE_IDDLE;
      state_changed = true;
    }
  }
  if (my_time_ms > hard_deadline_ms)
  {
    printf ("\n");
    if (new_strat_state!=STRAT_STATE_NULL)
      m_strat_state = new_strat_state;
    state_changed = true;
  }

  return state_changed;
}

bool RobotStrat::do_STRAT_STATE_INIT_ACTION(strat_action_t *my_action, bool send_prep_cmd)
{
  bool action_ok = false;

  switch (my_action->h.type) {
  case STRAT_ACTION_TYPE_WAIT:
    printf (" STRAT_ACTION_TYPE_WAIT\n");
    action_ok = true;
    break;
  case STRAT_ACTION_TYPE_STOP:
    printf (" STRAT_ACTION_TYPE_STOP\n");
    /* FIXME : TODO */
    printf (" Forcing IDDLE state!\n");
    action_ok = false;
    break;
  case STRAT_ACTION_TYPE_TRAJ:
    printf (" STRAT_ACTION_TYPE_TRAJ\n");
    action_ok = true;
    break;
  case STRAT_ACTION_TYPE_POINT_TO:
    printf (" STRAT_ACTION_TYPE_POINT_TO\n");
    action_ok = true;
    break;
  case STRAT_ACTION_TYPE_NUCLEO_SEQ:
    printf (" STRAT_ACTION_TYPE_NUCLEO_SEQ\n");
    action_ok = true;
    break;
  case STRAT_ACTION_TYPE_GOTO_ASTAR:
  {
    strat_action_goto_astar_t *act_ast= 
      (strat_action_goto_astar_t *) my_action;
    printf (" STRAT_ACTION_TYPE_GOTO_ASTAR\n");
    action_ok = true;
    /* clear playground */
    m_path_find_pg.erase_mob_obst();
    /* put mobile obstacles */
    WorldState::instance().lock();
    detected_robot_info_t& o0 = 
      WorldState::instance().detected_robot(0);
    detected_robot_info_t& o1 = 
      WorldState::instance().detected_robot(1);
    detected_robot_info_t& o2 = 
      WorldState::instance().detected_robot(2);
    WorldState::instance().release();
    if (o0.detect_quality>2)
      m_path_find_pg.put_mob_point_obst(o0.x_mm, o0.y_mm);
    if (o1.detect_quality>2)
      m_path_find_pg.put_mob_point_obst(o1.x_mm, o1.y_mm);
    if (o2.detect_quality>2)
      m_path_find_pg.put_mob_point_obst(o2.x_mm, o2.y_mm);
    float x_start_mm = RobotState::instance().s().x_mm;
    float y_start_mm = RobotState::instance().s().y_mm;
    m_path_find_pg.put_free_zone(x_start_mm, y_start_mm);
    /* apply A* */
    m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM,m_path_find_pg.Y_SZ_CM);
    m_path_find_pg.feed_astar(m_core_astar);
    int x_start_cm = x_start_mm/10;
    int y_start_cm = y_start_mm/10;
    int x_end_cm   = act_ast->target.x_mm/10;
    int y_end_cm   = act_ast->target.y_mm/10;
    int Y_OFF_CM   = m_path_find_pg.Y_OFFSET_CM;
    int X_SZ_CM    = m_path_find_pg.X_SZ_CM;
    printf ("START(cm) : <%d,%d>\n", x_start_cm, y_start_cm);
    printf ("END(cm) : <%d,%d>\n", x_end_cm, y_end_cm);
    m_core_astar.setWay(x_start_cm, y_start_cm+Y_OFF_CM, 1);
    m_core_astar.setWay(x_end_cm,   y_end_cm+Y_OFF_CM,   1);
    m_core_astar.setStart(x_start_cm, y_start_cm+Y_OFF_CM);
    m_core_astar.setEnd(x_end_cm, y_end_cm+Y_OFF_CM);
    m_core_astar.search();
    int x_wp = 0;
    int y_wp = 0;
    list<pair<UINT,UINT>> path = m_core_astar.getPath(AStarPathType::raw);
    if(path.size() > 0)
    {
      list<pair<UINT, UINT>>::iterator pathIt;
      for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
      {
        x_wp = pathIt->first;
        y_wp = pathIt->second;
        y_wp -= Y_OFF_CM;
        m_path_find_pg.m_playground[(y_wp+Y_OFF_CM)*(X_SZ_CM) + x_wp] = 
          m_path_find_pg.PATH;
      }
    }
    int wp_idx = 0;
    bool start_of_path = true;
    path = m_core_astar.getPath(AStarPathType::smooth);
    if(path.size() > 0)
    {
      if(path.size() == 1) /* FIXME : TODO : workaround Nucleo bug */
      {
        int x_wp_mm = RobotState::instance().s().x_mm;
        int y_wp_mm = RobotState::instance().s().y_mm;
        act_ast->wp[wp_idx].x_mm = x_wp_mm;
        act_ast->wp[wp_idx].y_mm = y_wp_mm;
        wp_idx++;
        act_ast->nwp = wp_idx;
        printf ("<%d,%d>\n",x_wp_mm,y_wp_mm);
        start_of_path = false;
      }
      list<pair<UINT, UINT>>::iterator pathIt;
      for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
      {
        int x_wp_mm;
        int y_wp_mm;
        x_wp = pathIt->first;
        y_wp = pathIt->second;
        y_wp -= Y_OFF_CM;
        m_path_find_pg.m_playground[(y_wp+Y_OFF_CM)*(X_SZ_CM) + x_wp] = 
          m_path_find_pg.PATH_WP;
        x_wp_mm = x_wp*10;
        y_wp_mm = y_wp*10;
        if (start_of_path)
        {
          int x_start_mm = RobotState::instance().s().x_mm;
          int y_start_mm = RobotState::instance().s().y_mm;

          if (goldo_dist(x_start_mm, y_start_mm, x_wp_mm, y_wp_mm)>1.0)
          {
            /* ASSERT(wp_idx==0) */
            act_ast->wp[wp_idx].x_mm = x_start_mm;
            act_ast->wp[wp_idx].y_mm = y_start_mm;
            wp_idx++;
            act_ast->nwp = wp_idx;
            printf ("<%d,%d>\n",x_start_mm,y_start_mm);
          }
        }
        start_of_path = false;
        if ((unsigned int)wp_idx<_countof(act_ast->wp))
        {
          act_ast->wp[wp_idx].x_mm = x_wp_mm;
          act_ast->wp[wp_idx].y_mm = y_wp_mm;
          wp_idx++;
          act_ast->nwp = wp_idx;
        }
        else 
        {
          /* FIXME : TODO : what to do? */
        }
        printf ("<%d,%d>\n",x_wp_mm,y_wp_mm);
      }
    }
    else
    {
      printf (" Cannot generate path!\n");
      action_ok = false;
    }
    /* dump result for debug */
    sprintf(m_dbg_fname,"dump_astar_act%d.ppm",m_task_dbg->m_curr_act_idx);
    m_path_find_pg.create_playground_ppm();
    m_path_find_pg.dump_playground_ppm(m_dbg_fname);
    m_path_find_pg.send_playground_ppm();
    if (action_ok && send_prep_cmd)
    {
      /* FIXME : TODO : configuration for speed, acc and dec.. */
      cmd_point_to (&(act_ast->wp[1]), 3.5, 10.0, 10.0);
    }
  }
  break;
  case STRAT_ACTION_TYPE_BRANCH:
    printf (" STRAT_ACTION_TYPE_BRANCH\n");
    action_ok = true;
    break;
#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  case STRAT_ACTION_TYPE_CRIDF2021:
    printf (" STRAT_ACTION_TYPE_CRIDF2021\n");
    action_ok = true;
    break;
#endif
  default:
    printf (" Warning : Unknown action type!\n");
    action_ok = false;
  } /* switch (my_action->h.type) */

  return action_ok;
}

void RobotStrat::do_STRAT_STATE_EXEC_ACTION(strat_action_t *my_action)
{
  switch (my_action->h.type) {
  case STRAT_ACTION_TYPE_WAIT:
    break;
  case STRAT_ACTION_TYPE_STOP:
    break;
  case STRAT_ACTION_TYPE_TRAJ:
  {
    strat_action_traj_t *act_traj= (strat_action_traj_t *) my_action;
    cmd_traj (act_traj->wp, act_traj->nwp, 
              act_traj->speed, act_traj->accel, act_traj->deccel);
  }
  break;
  case STRAT_ACTION_TYPE_POINT_TO:
  {
    strat_action_point_to_t *act_pt= 
      (strat_action_point_to_t *) my_action;
    cmd_point_to (&(act_pt->target), 
                  act_pt->speed, act_pt->accel, act_pt->deccel);
  }
  break;
  case STRAT_ACTION_TYPE_NUCLEO_SEQ:
  {
    strat_action_nucleo_seq_t *act_nucleo_seq= 
      (strat_action_nucleo_seq_t *) my_action;
    cmd_nucleo_seq (act_nucleo_seq->nucleo_seq_id);
  }
  break;
  case STRAT_ACTION_TYPE_GOTO_ASTAR:
  {
    strat_action_goto_astar_t *act_ast= 
      (strat_action_goto_astar_t *) my_action;
    cmd_traj (act_ast->wp, act_ast->nwp, 
              act_ast->speed, act_ast->accel, act_ast->deccel);
  }
  break;
  case STRAT_ACTION_TYPE_BRANCH:
    break;
#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  case STRAT_ACTION_TYPE_CRIDF2021:
    break;
#endif
  default:
    printf (" Warning : Unknown action type!\n");
  } /* switch (my_action->h.type) */
}

bool RobotStrat::is_point_forward(float _x_mm, float _y_mm)
{
  float my_x_mm = RobotState::instance().s().x_mm;
  float my_y_mm = RobotState::instance().s().y_mm;
  float my_theta_rad = RobotState::instance().s().theta_deg*M_PI/180.0;

  float delta_x_mm = _x_mm - my_x_mm;
  float delta_y_mm = _y_mm - my_y_mm;

  if ((delta_x_mm*cos(my_theta_rad) + delta_y_mm*sin(my_theta_rad))>0.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

detected_robot_info_t* RobotStrat::get_nearest_obst(int _area_code)
{
  double my_x_mm = RobotState::instance().s().x_mm;
  double my_y_mm = RobotState::instance().s().y_mm;
  WorldState::instance().lock();
  detected_robot_info_t& o0 = 
    WorldState::instance().detected_robot(0);
  detected_robot_info_t& o1 = 
    WorldState::instance().detected_robot(1);
  detected_robot_info_t& o2 = 
    WorldState::instance().detected_robot(2);
  WorldState::instance().release();
  double d_obst = 3000.0;
  double d0_mm = 3001.0;
  double d1_mm = 3001.0;
  double d2_mm = 3001.0;

  bool o0_fwd = is_point_forward(o0.x_mm, o0.y_mm);
  bool o1_fwd = is_point_forward(o1.x_mm, o1.y_mm);
  bool o2_fwd = is_point_forward(o2.x_mm, o2.y_mm);

  bool o0_trig = (_area_code==0) || ((_area_code>0) && (o0_fwd)) || ((_area_code<0) && (!o0_fwd));
  bool o1_trig = (_area_code==0) || ((_area_code>0) && (o1_fwd)) || ((_area_code<0) && (!o1_fwd));
  bool o2_trig = (_area_code==0) || ((_area_code>0) && (o2_fwd)) || ((_area_code<0) && (!o2_fwd));

  if (o0_trig && (o0.detect_quality>2))
  {
    d0_mm = goldo_dist(my_x_mm, my_y_mm, o0.x_mm, o0.y_mm);
  }
  if (o1_trig && (o1.detect_quality>2))
  {
    d1_mm = goldo_dist(my_x_mm, my_y_mm, o1.x_mm, o1.y_mm);
  }
  if (o2_trig && (o2.detect_quality>2))
  {
    d2_mm = goldo_dist(my_x_mm, my_y_mm, o2.x_mm, o2.y_mm);
  }

  detected_robot_info_t* obst = NULL;
  if (d0_mm<d_obst)
  {
    d_obst = d0_mm;
    obst = &o0;
  }
  if (d1_mm<d_obst)
  {
    d_obst = d1_mm;
    obst = &o1;
  }
  if (d2_mm<d_obst)
  {
    d_obst = d2_mm;
    obst = &o2;
  }

  return obst;
}

/* FIXME : TODO : configuration */
#define SAFE_DISTANCE_MM  380.0
bool RobotStrat::adversary_moved_away()
{
  detected_robot_info_t* obst = get_nearest_obst(m_last_move_forwards?1:-1);
  if (obst==NULL)
  {
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
    s_moved_away_reason = 1;
#endif
    return true;
  }

  float my_x_mm = RobotState::instance().s().x_mm;
  float my_y_mm = RobotState::instance().s().y_mm;

  float delta_x_mm = obst->x_mm - my_x_mm;
  float delta_y_mm = obst->y_mm - my_y_mm;

#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
  s_x_mm = my_x_mm;
  s_y_mm = my_y_mm;
  s_theta_rad = RobotState::instance().s().theta_deg*M_PI/180.0;
  s_obst_x_mm = obst->x_mm;
  s_obst_y_mm = obst->y_mm;
  s_delta_x_mm = delta_x_mm;
  s_delta_y_mm = delta_y_mm;
#endif

  if ((delta_x_mm*delta_x_mm + delta_y_mm*delta_y_mm)>(SAFE_DISTANCE_MM*SAFE_DISTANCE_MM))
  {
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
    s_moved_away_reason = 2;
#endif
    return true;
  }

  if (is_point_forward(obst->x_mm,obst->y_mm))
  { /* adversary before us */
    if (!m_last_move_forwards)
    {
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
      s_moved_away_reason = 3;
#endif
      return true;
    }
  }
  else
  { /* adversary behind us */
    if (m_last_move_forwards)
    {
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
      s_moved_away_reason = 4;
#endif
      return true;
    }
  }

#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
  s_moved_away_reason = 0;
#endif
  return false;
}

strat_action_t * RobotStrat::prepare_STRAT_STATE_EMERGENCY_MOVE_AWAY()
{
  double my_x_mm = RobotState::instance().s().x_mm;
  double my_y_mm = RobotState::instance().s().y_mm;
  detected_robot_info_t* obst = get_nearest_obst(0);

  if (obst==NULL)
  {
    strat_action_wait_t *act_wait = (strat_action_wait_t *)m_task_dbg->m_emergency_action_buf;
    memset (act_wait, 0, sizeof(strat_action_wait_t));
    act_wait->h.type = STRAT_ACTION_TYPE_WAIT;
    act_wait->h.min_duration_ms = 200;
    act_wait->h.max_duration_ms = 1000;
    return (strat_action_t *) act_wait;
  }

  double d_obst = goldo_dist(my_x_mm, my_y_mm, obst->x_mm, obst->y_mm);

  double move_away_x_mm = my_x_mm + m_task_dbg->m_move_away_dist_mm*(my_x_mm - obst->x_mm)/d_obst;
  double move_away_y_mm = my_y_mm + m_task_dbg->m_move_away_dist_mm*(my_y_mm - obst->y_mm)/d_obst;

  printf ("Obstacle detected at <%f,%f>\n", obst->x_mm, obst->y_mm);
  printf ("Moving away to <%f,%f>\n", move_away_x_mm, move_away_y_mm);

  strat_action_traj_t *act_move_away = (strat_action_traj_t *)m_task_dbg->m_emergency_action_buf;
  memset (act_move_away, 0, sizeof(strat_action_traj_t));
  act_move_away->h.type = STRAT_ACTION_TYPE_TRAJ;
  act_move_away->h.min_duration_ms = 200;
  act_move_away->h.max_duration_ms = 4000;
  act_move_away->speed = 0.4;
  act_move_away->accel = 0.4;
  act_move_away->deccel = 0.4;
  act_move_away->nwp = 2;
  act_move_away->wp[0].x_mm = my_x_mm;
  act_move_away->wp[0].y_mm = my_y_mm;
  act_move_away->wp[1].x_mm = move_away_x_mm;
  act_move_away->wp[1].y_mm = move_away_y_mm;

  return (strat_action_t *) act_move_away;
}

strat_action_t * RobotStrat::prepare_STRAT_STATE_EMERGENCY_ESCAPE()
{
  strat_action_goto_astar_t *act_escape = (strat_action_goto_astar_t *)m_task_dbg->m_emergency_action_buf;
  memset (act_escape, 0, sizeof(strat_action_goto_astar_t));
  act_escape->h.type = STRAT_ACTION_TYPE_GOTO_ASTAR;
  act_escape->h.min_duration_ms = 200;
  act_escape->h.max_duration_ms = 20000;
  act_escape->speed = 0.4;
  act_escape->accel = 0.4;
  act_escape->deccel = 0.4;
  act_escape->target = m_task_dbg->m_current_action_final_wp;

  return (strat_action_t *) act_escape;
}

strat_action_t * RobotStrat::prepare_STRAT_STATE_EMERGENCY_RECOVER(strat_action_t * orig_act)
{
  if (orig_act->h.type == STRAT_ACTION_TYPE_TRAJ)
  {
    strat_action_traj_t *act_traj = (strat_action_traj_t *) orig_act;

    int next_wp = 1;
    int last_wp = act_traj->nwp-1;

    float my_x_mm = RobotState::instance().s().x_mm;
    float my_y_mm = RobotState::instance().s().y_mm;
    float my_dist = 0.0;
    float min_dist = 3000.0;

#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
    printf ("DEBUG : prepare_STRAT_STATE_EMERGENCY_RECOVER() :\n");
    printf ("  my_pose : [%8.1f, %8.1f] \n", my_x_mm, my_y_mm);
    printf ("  original TRAJ:\n");
    printf ("    nwp : %d\n", act_traj->nwp);
    printf ("    wp:\n");
    for (int j=0; j<act_traj->nwp; j++)
    {
      printf ("      - [%8.1f,%8.1f]\n", 
                act_traj->wp[j].x_mm, act_traj->wp[j].y_mm);
    }
    printf ("  computing dists :\n");
#endif
    for (int i=1; i<=last_wp; i++)
    {
      my_dist = goldo_dist(my_x_mm,my_y_mm,act_traj->wp[i].x_mm,act_traj->wp[i].y_mm);
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
      printf ("    %d : %8.1f\n", i, my_dist);
#endif
      if (my_dist < min_dist)
      {
        next_wp = i;
        min_dist = my_dist;
      }
    }
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
    printf ("  last_wp : %d\n", last_wp);
    printf ("  next_wp : %d\n", next_wp);
    printf ("  min_dist : %8.1f\n", min_dist);
#endif

    if (next_wp < last_wp)
    {
      /* Pas de retour en arriere! */
      float delta_x = act_traj->wp[next_wp].x_mm - my_x_mm;
      float delta_y = act_traj->wp[next_wp].y_mm - my_y_mm;
      float delta_x_wp = act_traj->wp[next_wp+1].x_mm - act_traj->wp[next_wp].x_mm;
      float delta_y_wp = act_traj->wp[next_wp+1].y_mm - act_traj->wp[next_wp].y_mm;

      if ((delta_x*delta_x_wp+delta_y*delta_y_wp)<=0.0)
      {
#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
        printf ("  next_wp = %d was backwards, skipping to : %d\n", next_wp, next_wp+1);
#endif
        next_wp++;
      }

      act_traj->wp[0].x_mm = my_x_mm;
      act_traj->wp[0].y_mm = my_y_mm;
      int new_i=1;
      for (int i=next_wp; i<=last_wp; i++)
      {
        act_traj->wp[new_i].x_mm = act_traj->wp[i].x_mm;
        act_traj->wp[new_i].y_mm = act_traj->wp[i].y_mm;
        new_i++;
      }
      act_traj->nwp = new_i;
    }
    else
    {
      act_traj->wp[0].x_mm = my_x_mm;
      act_traj->wp[0].y_mm = my_y_mm;
      act_traj->wp[1].x_mm = act_traj->wp[last_wp].x_mm;
      act_traj->wp[1].y_mm = act_traj->wp[last_wp].y_mm;
      act_traj->nwp = 2;
    }

#ifdef DEBUG_OBST_AVOIDANCE /* FIXME : DEBUG */
    printf ("  recovered TRAJ:\n");
    printf ("    nwp : %d\n", act_traj->nwp);
    printf ("    wp:\n");
    for (int j=0; j<act_traj->nwp; j++)
    {
      printf ("      - [%8.1f,%8.1f]\n", 
                act_traj->wp[j].x_mm, act_traj->wp[j].y_mm);
    }
#endif
  }

  return orig_act;
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
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::PropulsionExecuteTrajectory;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
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
    float my_x = _wp[i].x_mm*0.001;
    float my_y = _wp[i].y_mm*0.001;

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
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::PropulsionExecutePointTo;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = _wp->x_mm*0.001;
  float my_y = _wp->y_mm*0.001;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
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

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_set_pose(float x_mm, float y_mm, float theta_deg)
{
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::PropulsionSetPose;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = x_mm*0.001;
  float my_y = y_mm*0.001;
  float my_theta = theta_deg*M_PI/180.0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
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
  memcpy (_pc, (unsigned char *)&my_theta, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_nucleo_seq (unsigned int seq_id)
{
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::MainSequenceStartSequence;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;
  unsigned short seq_id_s = (unsigned short) seq_id;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&(seq_id_s), field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_clear_prop_err()
{
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::PropulsionClearError;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_motors_disable()
{
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::SetMotorsEnable;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = 1;
  *_pc = 0x00; /* disable */
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_propulsion_disable()
{
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::PropulsionSetEnable;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  field_len = 1;
  *_pc = 0x00; /* disable */
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_emergency_stop()
{
  unsigned short int cmd_code = (unsigned short int ) goldobot::CommMessageType::CmdEmergencyStop;

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_code, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}


/******************************************************************************/
/**  DEBUG  *******************************************************************/
/******************************************************************************/

void RobotStrat::dbg_astar_test(int x_start_mm, int y_start_mm,
                                int x_end_mm, int y_end_mm,
                                int xo1_mm, int yo1_mm,
                                int xo2_mm, int yo2_mm,
                                int xo3_mm, int yo3_mm,
                                char *dump_fname)
{
  int x_start = x_start_mm/10;
  int y_start = y_start_mm/10;
  int x_end   = x_end_mm/10;
  int y_end   = y_end_mm/10;
  int x_wp = 0;
  int y_wp = 0;
  bool isNewPath = false;
  int Y_OFF_CM = m_path_find_pg.Y_OFFSET_CM;
  int X_SZ_CM = m_path_find_pg.X_SZ_CM;

  /* clear playground */
  m_path_find_pg.erase_mob_obst();

  /* put mobile obstacles */
  m_path_find_pg.put_mob_point_obst(xo1_mm, yo1_mm);
  m_path_find_pg.put_mob_point_obst(xo2_mm, yo2_mm);
  m_path_find_pg.put_mob_point_obst(xo3_mm, yo3_mm);

  /* put free zone around our robot initial position */
  m_path_find_pg.put_free_zone(x_start_mm, y_start_mm);

  /* astar test */
  m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);
  m_path_find_pg.feed_astar(m_core_astar);

  m_core_astar.setWay(x_start, y_start+Y_OFF_CM, 1);
  m_core_astar.setWay(x_end,   y_end+Y_OFF_CM,   1);

  m_core_astar.setStart(x_start, y_start+Y_OFF_CM);
  m_core_astar.setEnd(x_end, y_end+Y_OFF_CM);

  list<pair<UINT, UINT>> path= m_core_astar.getPathOnlyIfNeed(true, &isNewPath);
  printf ("path(OnlyIfNeed).size() = %d\n",(int)path.size());

  path = m_core_astar.getPath(AStarPathType::raw);
  printf ("path(raw).size() = %d\n",(int)path.size());
  if(path.size() > 0)
  {
    list<pair<UINT, UINT>>::iterator pathIt;
    for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
    {
      x_wp = pathIt->first;
      y_wp = pathIt->second;
      y_wp -= Y_OFF_CM;
      m_path_find_pg.m_playground[(y_wp+Y_OFF_CM)*(X_SZ_CM) + x_wp] = 
        m_path_find_pg.PATH;
    }
  }

  path = m_core_astar.getPath(AStarPathType::smooth);
  printf ("path(smooth).size() = %d\n",(int)path.size());
  if(path.size() > 0)
  {
    list<pair<UINT, UINT>>::iterator pathIt;
    for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
    {
      x_wp = pathIt->first;
      y_wp = pathIt->second;
      y_wp -= Y_OFF_CM;
      m_path_find_pg.m_playground[(y_wp+Y_OFF_CM)*(X_SZ_CM) + x_wp] = 
        m_path_find_pg.PATH_WP;
      printf ("<%d,%d>\n",x_wp,y_wp);
    }
  }

  /* dump result */
  m_path_find_pg.create_playground_ppm();
  m_path_find_pg.dump_playground_ppm(dump_fname);

  printf ("\n");
}

void RobotStrat::dbg_dump()
{
  read_yaml_conf (m_strat_file_name);
  m_task_dbg->dbg_dump_task();
}


/* FIXME : DEBUG : HACK CRIDF2021 + */
void TaskCRIDF2021::init(bool is_blue)
{
  m_side_is_blue = is_blue;
  m_task_state = TASK_STATE_IDDLE;
  m_state_change = false;
  memset (&m_target, 0, sizeof(m_target));
  if (is_blue)
  {
    m_harbor.x_mm  = 1600;
    m_harbor.y_mm  =  300;
    m_obs_pt.x_mm  =  800;
    m_obs_pt.y_mm  =  600;
    m_obs_dir.x_mm = 1000;
    m_obs_dir.y_mm =    0;
  }
  else
  {
    m_harbor.x_mm  = 1600;
    m_harbor.y_mm  = -300;
    m_obs_pt.x_mm  =  800;
    m_obs_pt.y_mm  = -600;
    m_obs_dir.x_mm = 1000;
    m_obs_dir.y_mm =    0;
  }
  m_soft_deadline_ms = 500;   /* FIXME : DEBUG */
  m_hard_deadline_ms = 10000; /* FIXME : DEBUG */
}

void TaskCRIDF2021::set_state(task_state_cridf2021_t new_state, unsigned int _time_ms)
{
  m_task_state = new_state;
  m_act_start_time_ms = _time_ms;
  m_state_change = true;
}

bool TaskCRIDF2021::state_exit_test(unsigned int _time_ms,
                                    unsigned int soft_deadline_ms,
                                    unsigned int hard_deadline_ms)
{
  bool exit_test = false;

  if (_time_ms > m_act_start_time_ms + soft_deadline_ms)
  {
    /* FIXME : TODO : add completion test for actuators */
    if (!RobotState::instance().propulsion_busy())
    {
      exit_test = true;
    }
    /* FIXME : TODO : error management */
    //else if (RobotState::instance().propulsion_error())
    //{
    //  printf (" Propulsion ERROR\n");
    //  m_strat_state = TASK_STATE_IDDLE;
    //  m_state_change = false;
    //  exit_test = true;
    //}
  }
  if (_time_ms > m_act_start_time_ms + hard_deadline_ms)
  {
    exit_test = true;
  }

  return exit_test;
}

void TaskCRIDF2021::check_deadlines_and_change_state(unsigned int _time_ms,
                                                     task_state_cridf2021_t new_state)
{
  if (state_exit_test(_time_ms, m_soft_deadline_ms, m_hard_deadline_ms))
  {
    set_state(new_state, _time_ms);
  }
}

strat_action_traj_t * TaskCRIDF2021::prepare_action_go_to(strat_way_point_t *_target)
{
  double my_x_mm = RobotState::instance().s().x_mm;
  double my_y_mm = RobotState::instance().s().y_mm;

  strat_action_traj_t *act_goto = (strat_action_traj_t *)m_action_buf;
  memset (act_goto, 0, sizeof(strat_action_traj_t));
  act_goto->h.type = STRAT_ACTION_TYPE_TRAJ;
  act_goto->h.min_duration_ms = 500;
  act_goto->h.max_duration_ms = 10000;
  act_goto->speed = 0.4;
  act_goto->accel = 0.4;
  act_goto->deccel = 0.4;
  act_goto->nwp = 2;
  act_goto->wp[0].x_mm = my_x_mm;
  act_goto->wp[0].y_mm = my_y_mm;
  act_goto->wp[1].x_mm = _target->x_mm;
  act_goto->wp[1].y_mm = _target->y_mm;

  return act_goto;
}

strat_action_point_to_t * TaskCRIDF2021::prepare_action_point_to(strat_way_point_t *_target)
{
  strat_action_point_to_t *act_point = (strat_action_point_to_t *)m_action_buf;
  memset (act_point, 0, sizeof(strat_action_point_to_t));
  act_point->h.min_duration_ms = 500;
  act_point->h.max_duration_ms = 10000;
  act_point->speed = 3.5;
  act_point->accel = 10.0;
  act_point->deccel = 10.0;
  act_point->target.x_mm = _target->x_mm;
  act_point->target.y_mm = _target->y_mm;

  return act_point;
}

void TaskCRIDF2021::do_step(float _time_ms)
{

  switch (m_task_state) {
  case TASK_STATE_IDDLE:
    if(m_state_change)
    {
      /* FIXME : TODO : what to do? */
      m_state_change = false;
    }
    //check_deadlines_and_change_state(_time_ms, TASK_STATE_GO_TO_OBSERVATION_POINT);
    /* FIXME : TODO : etat final ou non? */
    break;
  case TASK_STATE_GO_TO_OBSERVATION_POINT:
    if(m_state_change)
    {
      strat_action_traj_t * goto_act = prepare_action_go_to(&m_obs_pt);
      RobotStrat::instance().cmd_traj (goto_act->wp, goto_act->nwp, 
                                       goto_act->speed, goto_act->accel, goto_act->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_POINT_TO_PLAYGROUND_CENTER);
    break;
  case TASK_STATE_POINT_TO_PLAYGROUND_CENTER:
    if(m_state_change)
    {
      strat_action_point_to_t * act_pt = prepare_action_point_to(&m_obs_dir);
      RobotStrat::instance().cmd_point_to (&(act_pt->target), 
                                           act_pt->speed, act_pt->accel, act_pt->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_GET_TARGET);
    break;
  case TASK_STATE_GET_TARGET:
    if(m_state_change)
    {
#if 1 /* FIXME : DEBUG : TEST */
      m_target.timestamp_ms = _time_ms;
      m_target.id = 1;
      m_target.attr = 2; /* GREEN */
      m_target.x_mm = 800;
      m_target.y_mm = 400;
#endif
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_POINT_TO_TARGET);
    break;
  case TASK_STATE_POINT_TO_TARGET:
    if(m_state_change)
    {
      strat_way_point_t m_target_pose;
      m_target_pose.x_mm = m_target.x_mm;
      m_target_pose.y_mm = m_target.y_mm;
      strat_action_point_to_t * act_pt = prepare_action_point_to(&m_target_pose);
      RobotStrat::instance().cmd_point_to (&(act_pt->target), 
                                           act_pt->speed, act_pt->accel, act_pt->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_GO_TO_TARGET);
    break;
  case TASK_STATE_GO_TO_TARGET:
    if(m_state_change)
    {
      RobotStrat::instance().cmd_nucleo_seq (14); /* pales_ouvre */
      strat_way_point_t m_target_pose;
      m_target_pose.x_mm = m_target.x_mm;
      m_target_pose.y_mm = m_target.y_mm;
      strat_action_traj_t * goto_act = prepare_action_go_to(&m_target_pose);
      RobotStrat::instance().cmd_traj (goto_act->wp, goto_act->nwp, 
                                       goto_act->speed, goto_act->accel, goto_act->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_CATCH_TARGET);
    break;
  case TASK_STATE_CATCH_TARGET:
    if(m_state_change)
    {
      RobotStrat::instance().cmd_nucleo_seq (15); /* pales_serre */
      m_soft_deadline_ms = 1000; /* FIXME : DEBUG */
      m_hard_deadline_ms = 2000; /* FIXME : DEBUG */
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_POINT_TO_HARBOR);
    break;
  case TASK_STATE_POINT_TO_HARBOR:
    if(m_state_change)
    {
      strat_action_point_to_t * act_pt = prepare_action_point_to(&m_harbor);
      RobotStrat::instance().cmd_point_to (&(act_pt->target), 
                                           act_pt->speed, act_pt->accel, act_pt->deccel);
      m_soft_deadline_ms = 500;   /* FIXME : DEBUG */
      m_hard_deadline_ms = 10000; /* FIXME : DEBUG */
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_GO_TO_HARBOR);
    break;
  case TASK_STATE_GO_TO_HARBOR:
    if(m_state_change)
    {
      strat_action_traj_t * goto_act = prepare_action_go_to(&m_harbor);
      RobotStrat::instance().cmd_traj (goto_act->wp, goto_act->nwp, 
                                       goto_act->speed, goto_act->accel, goto_act->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_ENTER_HARBOR);
    break;
  case TASK_STATE_ENTER_HARBOR:
    if(m_state_change)
    {
      RobotStrat::instance().cmd_nucleo_seq (14); /* pales_ouvre */
      strat_way_point_t inside_harbor;
      inside_harbor.x_mm = m_harbor.x_mm + 200;
      if (m_target.attr == 1) /* RED */
      {
        inside_harbor.y_mm = m_harbor.y_mm + 100;
      }
      else /* GREEN */
      {
        inside_harbor.y_mm = m_harbor.y_mm - 100;
      }
      strat_action_traj_t * goto_act = prepare_action_go_to(&inside_harbor);
      RobotStrat::instance().cmd_traj (goto_act->wp, goto_act->nwp, 
                                       goto_act->speed, goto_act->accel, goto_act->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_EXIT_HARBOR);
    break;
  case TASK_STATE_EXIT_HARBOR:
    if(m_state_change)
    {
      strat_action_traj_t * goto_act = prepare_action_go_to(&m_harbor);
      RobotStrat::instance().cmd_traj (goto_act->wp, goto_act->nwp, 
                                       goto_act->speed, goto_act->accel, goto_act->deccel);
      m_state_change = false;
    }
    check_deadlines_and_change_state(_time_ms, TASK_STATE_GO_TO_OBSERVATION_POINT);
    break;
  case TASK_STATE_EMERGENCY_STOP:
    if(m_state_change)
    {
      /* FIXME : TODO */
      m_state_change = false;
    }
#if 0 /* FIXME : DEBUG */
    check_deadlines_and_change_state(_time_ms, TASK_STATE_EMERGENCY_WAIT);
#else
    m_task_state = TASK_STATE_IDDLE;
    m_state_change = false;
#endif
    break;
  case TASK_STATE_EMERGENCY_WAIT:
    if(m_state_change)
    {
      /* FIXME : TODO */
      m_state_change = false;
    }
#if 0 /* FIXME : DEBUG */
    check_deadlines_and_change_state(_time_ms, TASK_STATE_EMERGENCY_MOVE_AWAY);
#else
    m_task_state = TASK_STATE_IDDLE;
    m_state_change = false;
#endif
    break;
  case TASK_STATE_EMERGENCY_MOVE_AWAY:
    if(m_state_change)
    {
      /* FIXME : TODO */
      m_state_change = false;
    }
#if 0 /* FIXME : DEBUG */
    check_deadlines_and_change_state(_time_ms, TASK_STATE_EMERGENCY_ESCAPE);
#else
    m_task_state = TASK_STATE_IDDLE;
    m_state_change = false;
#endif
    break;
  case TASK_STATE_EMERGENCY_ESCAPE:
    if(m_state_change)
    {
      /* FIXME : TODO */
      m_state_change = false;
    }
#if 0 /* FIXME : DEBUG */
    check_deadlines_and_change_state(_time_ms, TASK_STATE_GO_TO_OBSERVATION_POINT);
#else
    m_task_state = TASK_STATE_IDDLE;
    m_state_change = false;
#endif
    break;
  default:
    m_task_state = TASK_STATE_IDDLE;
    m_state_change = false;
  }

}


/* FIXME : DEBUG : HACK CRIDF2021 - */


