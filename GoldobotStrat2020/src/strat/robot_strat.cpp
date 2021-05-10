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

  //m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);


  /* DEBUG */
  m_task_dbg = new StratTask();

  m_dbg_step_by_step = false;

  m_dbg_pause_match_sig = false;

  m_dbg_resume_match_sig = false;

  memset (m_dbg_fname, 0, sizeof(m_dbg_fname));

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

  m_path_find_pg.init();

  m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);

  read_yaml_conf (m_strat_file_name);


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

  while(!m_stop_task)
  {
    struct timespec my_tp;
    unsigned int my_time_ms = 0;
    bool action_ok = false;

    clock_gettime(1, &my_tp);
    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

    if (!m_dbg_step_by_step)
    {
      if ((!match_funny_done) && (match_start_ms!=0) && (my_time_ms>(match_start_ms+95000)))
      {
        /* FIXME : TODO */
        //cmd_nucleo_seq (9);
        printf ("\n");
        printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf ("! DON'T FORGET THE FUNNY ACTION !!!!!!!!\n");
        printf ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf ("\n");
        match_funny_done = true;
      }

      if ((match_start_ms!=0) && (my_time_ms>(match_start_ms+100000)) && (m_strat_state != STRAT_STATE_IDDLE))
      {
        RobotState::instance().s().strat_stop = true;
        cmd_emergency_stop();
        m_strat_state = STRAT_STATE_IDDLE;
        state_change_dbg = true;
      }
    }

    if ((match_start_ms!=0) && (my_time_ms<(match_start_ms+100000)))
    {
      sprintf (log_lidar_write_buf,"%d %d\n",my_time_ms-match_start_ms,RobotState::instance().s().obstacle_plot_cnt);
      write (log_lidar_fd,log_lidar_write_buf,strlen(log_lidar_write_buf));
    }

    if (RobotState::instance().emergency_stop() && 
        (m_strat_state!=STRAT_STATE_EMERGENCY_STOP) &&
        (m_strat_state!=STRAT_STATE_EMERGENCY_MOVE_AWAY) )
    {
      printf ("EMERGENCY_STOP!\n");
      cmd_clear_prop_err();
      /* FIXME : TODO : is this necessary? */
      RobotState::instance().m_lidar_detection_enabled = false;
      RobotState::instance().set_obstacle_gpio(false);
      hard_deadline_ms = my_time_ms + 500; /* FIXME : TODO : configuration.. */
      m_strat_state = STRAT_STATE_EMERGENCY_STOP;
      state_change_dbg = true;
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

      /* FIXME : DEBUG */
      m_task_dbg->m_curr_act_idx = 0;

      if ((!RobotState::instance().tirette_present()) || m_start_match_sig)
      {
        match_start_ms = my_time_ms;

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

      action_ok = do_STRAT_STATE_INIT_ACTION(my_action);

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

      do_STRAT_STATE_EXEC_ACTION(my_action);

      m_strat_state = STRAT_STATE_WAIT_END_ACTION;
      state_change_dbg = true;

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

    case STRAT_STATE_END_ACTION:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_END_ACTION ***************\n");
        printf ("****************************************\n");
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

      if (my_time_ms > hard_deadline_ms)
      {
        printf ("\n");
        auto move_away_action = prepare_STRAT_STATE_EMERGENCY_MOVE_AWAY();
        do_STRAT_STATE_INIT_ACTION(move_away_action);
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
        action_ok = do_STRAT_STATE_INIT_ACTION(my_escape_action);
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

bool RobotStrat::do_STRAT_STATE_INIT_ACTION(strat_action_t *my_action)
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
    /* apply A* */
    m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM,m_path_find_pg.Y_SZ_CM);
    m_path_find_pg.feed_astar(m_core_astar);
    int x_start_cm = RobotState::instance().s().x_mm/10;
    int y_start_cm = RobotState::instance().s().y_mm/10;
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
    if (action_ok)
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
  default:
    printf (" Warning : Unknown action type!\n");
  } /* switch (my_action->h.type) */
}

strat_action_t * RobotStrat::prepare_STRAT_STATE_EMERGENCY_MOVE_AWAY()
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
  double d0_mm = goldo_dist(my_x_mm, my_y_mm, o0.x_mm, o0.y_mm);
  double d1_mm = goldo_dist(my_x_mm, my_y_mm, o1.x_mm, o1.y_mm);
  double d2_mm = goldo_dist(my_x_mm, my_y_mm, o2.x_mm, o2.y_mm);

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

  if (obst==NULL)
  {
    strat_action_wait_t *act_wait = (strat_action_wait_t *)m_task_dbg->m_emergency_action_buf;
    memset (act_wait, 0, sizeof(strat_action_wait_t));
    act_wait->h.type = STRAT_ACTION_TYPE_WAIT;
    act_wait->h.min_duration_ms = 200;
    act_wait->h.max_duration_ms = 1000;
    return (strat_action_t *) act_wait;
  }

  double move_away_x_mm = my_x_mm + 200.0*(my_x_mm - obst->x_mm)/d_obst;
  double move_away_y_mm = my_y_mm + 200.0*(my_y_mm - obst->y_mm)/d_obst;

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
  m_task_dbg->dbg_dump_task();
}



