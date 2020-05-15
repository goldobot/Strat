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

  while(!m_stop_task)
  {
    struct timespec my_tp;
    unsigned int my_time_ms = 0;
    bool action_ok = false;

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
      m_task_dbg->m_curr_act_idx = 0;

      if ((!RobotState::instance().tirette_present()) || m_start_match_sig)
      {
        printf ("\n DEBUG : GO!..\n\n");

        /* FIXME : TODO : necessary? */
#ifndef WIN32
        usleep(100000);
#else
        Sleep(100);
#endif

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

      if (m_task_dbg->m_curr_act_idx < m_task_dbg->m_n_actions)
      {
        my_action = m_task_dbg->m_action_list[m_task_dbg->m_curr_act_idx];

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

      switch (my_action->h.type) {
      case STRAT_ACTION_TYPE_WAIT:
        printf (" STRAT_ACTION_TYPE_WAIT\n");
        action_ok = true;
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
              if (wp_idx<_countof(act_ast->wp))
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
        }
        break;
      default:
        printf (" Warning : Unknown action type!\n");
        action_ok = false;
      } /* switch (my_action->h.type) */

      if (action_ok)
      {
        if (m_dbg_step_by_step)
        {
          m_strat_state = STRAT_STATE_PAUSE2_DBG;
        }
        else
        {
          m_strat_state = STRAT_STATE_EXEC_ACTION_DBG;
        }
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

      soft_deadline_ms = my_time_ms + my_action->h.min_duration_ms;
      hard_deadline_ms = my_time_ms + my_action->h.max_duration_ms;

      switch (my_action->h.type) {
      case STRAT_ACTION_TYPE_WAIT:
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
          /* execute action! */
          cmd_traj (act_ast->wp, act_ast->nwp, 
                    act_ast->speed, act_ast->accel, act_ast->deccel);
        }
        break;
      default:
        printf (" Warning : Unknown action type!\n");
      } /* switch (my_action->h.type) */

      m_strat_state = STRAT_STATE_WAIT_END_ACTION_DBG;
      state_change_dbg = true;

      break;

    case STRAT_STATE_WAIT_END_ACTION_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_WAIT_END_ACTION_DBG ******\n");
        printf ("****************************************\n");
        printf ("\n");
        state_change_dbg = false;
      }

      if (my_time_ms > soft_deadline_ms)
      {
        if (my_time_ms < (soft_deadline_ms+20)) printf (".");
        /* FIXME : TODO : add completion test for actuators */
        if (!RobotState::instance().propulsion_busy())
        {
          printf ("\n");
          printf (" Action DONE\n");
          m_strat_state = STRAT_STATE_END_ACTION_DBG;
          state_change_dbg = true;
        }
        /* FIXME : TODO : error management */
        else if (RobotState::instance().propulsion_error())
        {
          printf ("\n");
          printf (" Propulsion ERROR\n");
          m_strat_state = STRAT_STATE_IDDLE;
          state_change_dbg = true;
        }
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
        m_strat_state = STRAT_STATE_GET_ACTION_DBG;
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
        m_strat_state = STRAT_STATE_EXEC_ACTION_DBG;
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
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0058; /*DbgPropulsionExecutePointTo*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = _wp->x_mm*0.001;
  float my_y = _wp->y_mm*0.001;

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

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_set_pose(float x_mm, float y_mm, float theta_deg)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0053; /*DbgPropulsionSetPose*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = x_mm*0.001;
  float my_y = y_mm*0.001;
  float my_theta = theta_deg*M_PI/180.0;

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
  memcpy (_pc, (unsigned char *)&my_theta, field_len);
  _pc += field_len;
  cmd_buf_len += field_len;

  DirectUartNucleo::instance().send(m_nucleo_cmd_buf, cmd_buf_len);

  return 0;
}

int RobotStrat::cmd_nucleo_seq (unsigned int seq_id)
{
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x002b; /*MainSequenceStartSequence*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;
  unsigned short seq_id_s = (unsigned short) seq_id;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
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
  /* FIXME : TODO : use defines.. */
  unsigned short int cmd_traj_code = 0x0063; /*PropulsionClearError*/

  unsigned char *_pc = m_nucleo_cmd_buf;
  int cmd_buf_len = 0;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy (_pc, (unsigned char *)&cmd_traj_code, field_len);
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



