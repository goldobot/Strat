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
/**  Playground  **************************************************************/
/******************************************************************************/

StratPlayground::StratPlayground()
{
  memset (m_stat_pattern, 0, sizeof(m_stat_pattern));
  memset (m_mob_pattern, 0, sizeof(m_mob_pattern));
  memset (m_playground, 0, sizeof(m_playground));
  memset (m_stat_playground, 0, sizeof(m_stat_playground));
}

void StratPlayground::init()
{
  int x, y;

  /* patterns */
  init_pattern(m_stat_pattern, S_PATT_SZ_CM, S_OBST_R_CM, S_EXCL);
  init_pattern(m_mob_pattern, M_PATT_SZ_CM, M_OBST_R_CM, M_EXCL);

  /* playground */
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    for (x=X_MIN_CM; x<X_MAX_CM; x++)
    {
      m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = NO_OBST;
    }
  }

  /* playground borders */
  x = X_MIN_CM;
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  x = X_MAX_CM-1;
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  y = Y_MIN_CM;
  for (x=X_MIN_CM; x<X_MAX_CM; x++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  y = Y_MAX_CM-1;
  for (x=X_MIN_CM; x<X_MAX_CM; x++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  /* static obstacles */
  put_stat_rect_obst(1850, 2000,  -610,  -590);
  put_stat_rect_obst(1850, 2000,   610,   590);
  put_stat_rect_obst(1700, 2000,   -10,    10);

  /* make backup */
  memcpy(m_stat_playground, m_playground, sizeof(m_playground));
}

void StratPlayground::init_pattern(unsigned char *_patt, 
                                   int _patt_sz_cm, int _obst_r_cm, 
                                   unsigned char _code)
{
  int x, y;
  int _obst_r2_cm = _obst_r_cm*_obst_r_cm;

  for (y=0; y<_patt_sz_cm; y++)
  {
    for (x=0; x<_patt_sz_cm; x++)
    {
      if (((x-_obst_r_cm)*(x-_obst_r_cm)+(y-_obst_r_cm)*(y-_obst_r_cm))<
          _obst_r2_cm)
      {
        _patt[(y)*(_patt_sz_cm) + x] = _code;
      }
      else
      {
        _patt[(y)*(_patt_sz_cm) + x] = NO_OBST;
      }
    }
  }

}

void StratPlayground::put_pattern(int x_cm, int y_cm, unsigned char*_patt, 
                                  int _patt_sz_cm)
{
  int xx, yy;
  int xx_abs, yy_abs;
  int _patt_sz_2_cm = _patt_sz_cm/2;

  for (yy=0; yy<_patt_sz_cm; yy++)
  {
    for (xx=0; xx<_patt_sz_cm; xx++)
    {
      xx_abs = x_cm-_patt_sz_2_cm+xx;
      yy_abs = y_cm-_patt_sz_2_cm+yy;
      if ((yy_abs>Y_MIN_CM) && (yy_abs<Y_MAX_CM) && 
          (xx_abs>X_MIN_CM) && (xx_abs<X_MAX_CM))
      {
        if (m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)]==NO_OBST)
          m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)] = 
            _patt[(yy)*(_patt_sz_cm) + xx];
      }
    }
  }
}

void StratPlayground::put_stat_rect_obst(int x_min_mm, int x_max_mm,
                                         int y_min_mm, int y_max_mm)
{
  int x, y;
  int x_min_cm = x_min_mm/10;
  int y_min_cm = y_min_mm/10;
  int x_max_cm = x_max_mm/10;
  int y_max_cm = y_max_mm/10;

  if (x_max_cm<x_min_cm)
  {
    x = x_min_cm;
    x_min_cm = x_max_cm;
    x_max_cm = x;
  }

  if (y_max_cm<y_min_cm)
  {
    y = y_min_cm;
    y_min_cm = y_max_cm;
    y_max_cm = y;
  }

  for (y=y_min_cm; y<y_max_cm; y++)
  {
    for (x=x_min_cm; x<x_max_cm; x++)
    {
      m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
      put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
    }
  }
}

void StratPlayground::put_mob_point_obst(int x_mm, int y_mm)
{
  int x = x_mm/10;
  int y = y_mm/10;

  m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = M_OBST;
  put_pattern(x, y, m_mob_pattern, M_PATT_SZ_CM);
}

void StratPlayground::feed_astar(AStar & _astar)
{
  int x;
  int y;

  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    for (x=X_MIN_CM; x<X_MAX_CM; x++)
    {
      unsigned char code = m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x];
      switch (code) {
      case NO_OBST:
      case OLD_PATH:
      case PATH_WP:
      case PATH:
        _astar.setWay(x, y+Y_OFFSET_CM, 1);
        break;
      case M_OBST:
      case M_EXCL:
        _astar.setWall(x, y+Y_OFFSET_CM);
        break;
      case S_OBST:
      case S_EXCL:
        _astar.setWall(x, y+Y_OFFSET_CM);
        break;
      }
    }
  }
}

void StratPlayground::erase_mob_obst()
{
  memcpy(m_playground, m_stat_playground, sizeof(m_playground));
}

void StratPlayground::dump_playground_ppm(char *ppm_fname)
{
  const int dimH = 300, dimV = 200;
  int i, j;
  FILE *fp = fopen(ppm_fname, "wb"); /* b - binary mode */
  (void) fprintf(fp, "P6\n%d %d\n255\n", dimH, dimV);
  for (j = 0; j < dimV; ++j)
  {
    for (i = 0; i < dimH; ++i)
    {
      unsigned char color[3];
      //color[0] = i % 256;  /* red */
      //color[1] = j % 256;  /* green */
      //color[2] = (i * j) % 256;  /* blue */

      unsigned char code = m_playground[(i)*(X_SZ_CM) + j];

      switch (code) {
      case NO_OBST:
        color[0] = 255;  /* red */
        color[1] = 255;  /* green */
        color[2] = 255;  /* blue */
        break;
      case S_OBST:
      case M_OBST:
        color[0] =   0;  /* red */
        color[1] =   0;  /* green */
        color[2] =   0;  /* blue */
        break;
      case S_EXCL:
      case M_EXCL:
        color[0] = 128;  /* red */
        color[1] = 128;  /* green */
        color[2] = 128;  /* blue */
        break;
      case OLD_PATH:
        color[0] = 255;  /* red */
        color[1] = 255;  /* green */
        color[2] = 128;  /* blue */
        break;
      case PATH_WP:
        color[0] =   0;  /* red */
        color[1] =   0;  /* green */
        color[2] = 255;  /* blue */
        break;
      case PATH:
        color[0] = 255;  /* red */
        color[1] = 255;  /* green */
        color[2] =   0;  /* blue */
        break;
      }

      (void) fwrite(color, 1, 3, fp);
    }
  }
  (void) fclose(fp);
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

  //m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);


  /* DEBUG */
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


  /* DEBUG */
  m_dbg_step_by_step = false;

  m_dbg_pause_match_sig = false;

  m_dbg_resume_match_sig = false;

  m_task_dbg.init_dbg();

  memset (m_dbg_fname, 0, sizeof(m_dbg_fname));

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
  usleep (100000);

  /* FIXME : DEBUG */
  m_dbg_step_by_step = true;

  while(!m_stop_task)
  {
    struct timespec my_tp;
    unsigned int my_time_ms = 0;
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

      if (((my_robot_sensors&2)==0) || m_start_match_sig)
      {
        printf ("\n DEBUG : Tirette!..\n\n");

        /* FIXME : TODO : necessary? */
        usleep (3000000);

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
        action_ok = false;
        /* FIXME : TODO */
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
          DetectedRobot& o0 = LidarDetect::instance().get_detected_mob_obst(0);
          m_path_find_pg.put_mob_point_obst(o0.x_mm, o0.y_mm);
          DetectedRobot& o1 = LidarDetect::instance().get_detected_mob_obst(1);
          m_path_find_pg.put_mob_point_obst(o1.x_mm, o1.y_mm);
          DetectedRobot& o2 = LidarDetect::instance().get_detected_mob_obst(2);
          m_path_find_pg.put_mob_point_obst(o2.x_mm, o2.y_mm);
          /* apply A* */
          m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM,m_path_find_pg.Y_SZ_CM);
          m_path_find_pg.feed_astar(m_core_astar);
          int x_start_cm = RobotState::instance().m_x_mm/10;
          int y_start_cm = RobotState::instance().m_y_mm/10;
          int x_end_cm   = act_ast->target.x_mm/10;
          int y_end_cm   = act_ast->target.y_mm/10;
          int Y_OFF_CM   = m_path_find_pg.Y_OFFSET_CM;
          int X_SZ_CM    = m_path_find_pg.X_SZ_CM;
          bool isNewPath = false;
          m_core_astar.setWay(x_start_cm, y_start_cm+Y_OFF_CM, 1);
          m_core_astar.setWay(x_end_cm,   y_end_cm+Y_OFF_CM,   1);
          m_core_astar.setStart(x_start_cm, y_start_cm+Y_OFF_CM);
          m_core_astar.setEnd(x_end_cm, y_end_cm+Y_OFF_CM);
          list<pair<UINT, UINT>> path= m_core_astar.getPathOnlyIfNeed(true, &isNewPath);
          int x_wp = 0;
          int y_wp = 0;
          path = m_core_astar.getPath(AStarPathType::raw);
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
              int x_wp_mm = RobotState::instance().m_x_mm;
              int y_wp_mm = RobotState::instance().m_y_mm;
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
          sprintf(m_dbg_fname,"dump_astar_act%d.ppm",m_task_dbg.m_curr_act_idx);
          m_path_find_pg.dump_playground_ppm(m_dbg_fname);
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
        printf (" STRAT_ACTION_TYPE_NUCLEO_SEQ\n");
        /* FIXME : TODO */
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

    case STRAT_STATE_PAUSE2_DBG:
      if (state_change_dbg)
      {
        printf ("\n");
        printf ("****************************************\n");
        printf ("* STRAT_STATE_PAUSE2_DBG ***************\n");
        printf ("****************************************\n");
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



/******************************************************************************/
/**  DEBUG  *******************************************************************/
/******************************************************************************/

strat_way_point_t dbg_point_to_point0 = {556.0, -1133.0};

strat_way_point_t dbg_traj1[] = {
  {  556.0, -1133.0},
  {  336.0, - 990.0},
  {  233.0, - 950.0},
  {  223.0, - 726.0},
  {  626.0, - 396.0},
  { 1013.0, - 470.0},
  { 1280.0, - 096.0},
  { 1153.0,   336.0},
  { 1300.0,   440.0},
  { 1490.0,   240.0},
  { 1740.0,   260.0},
};

strat_way_point_t dbg_traj2[] = {
  { 1740.0,   290.0},
  { 1490.0,   300.0},
};

strat_way_point_t dbg_point_to_point3 = {1400.0, -300.0};

strat_way_point_t dbg_target_point4 = {1850.0, -1350.0};

strat_way_point_t dbg_point_to_point5 = {1850.0, -790.0};

strat_way_point_t dbg_traj6[] = {
  { 1850.0, -1350.0},
  { 1850.0,  -790.0},
};

strat_way_point_t dbg_point_to_point7 = {300.0, -790.0};

strat_way_point_t dbg_target_point8 = {300.0, -790.0};

strat_way_point_t dbg_point_to_point9 = {300.0, -1350.0};

strat_way_point_t dbg_target_point10 = {300.0, -1350.0};

strat_way_point_t dbg_point_to_point11 = {300.0, -790.0};

int StratTask::init_dbg()
{
  strncpy(m_task_name,"DebugTask",sizeof(m_task_name));
  m_n_actions = 0;
  m_curr_act_idx = 0;
  memset (m_action_list, 0, sizeof(m_action_list));
  memset (m_action_buf, 0, sizeof(m_action_buf));
  m_priority = 0;
  m_started = false;
  m_completed = false;

  unsigned char *curr_act_p = m_action_buf;

  strat_action_point_to_t *action0 = (strat_action_point_to_t *) curr_act_p;
  action0->h.type = STRAT_ACTION_TYPE_POINT_TO;
  action0->h.min_duration_ms = 200;
  action0->h.max_duration_ms = 6000;
  action0->speed = 0.4;
  action0->accel = 0.3;
  action0->deccel = 0.3;
  action0->target = dbg_point_to_point0;
  m_action_list[m_n_actions] = (strat_action_t *) action0;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_point_to_t);

  strat_action_traj_t *action1 = (strat_action_traj_t *) curr_act_p;
  action1->h.type = STRAT_ACTION_TYPE_TRAJ;
  action1->h.min_duration_ms = 200;
  action1->h.max_duration_ms = 20000;
  action1->speed = 0.3;
  action1->accel = 0.2;
  action1->deccel = 0.2;
  action1->nwp = _countof(dbg_traj1);
  memcpy (action1->wp, dbg_traj1, sizeof(dbg_traj1));
  m_action_list[m_n_actions] = (strat_action_t *) action1;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_traj_t);

  strat_action_traj_t *action2 = (strat_action_traj_t *) curr_act_p;
  action2->h.type = STRAT_ACTION_TYPE_TRAJ;
  action2->h.min_duration_ms = 200;
  action2->h.max_duration_ms = 20000;
  action2->speed = 0.3;
  action2->accel = 0.2;
  action2->deccel = 0.2;
  action2->nwp = _countof(dbg_traj2);
  memcpy (action2->wp, dbg_traj2, sizeof(dbg_traj2));
  m_action_list[m_n_actions] = (strat_action_t *) action2;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_traj_t);

  strat_action_point_to_t *action3 = (strat_action_point_to_t *) curr_act_p;
  action3->h.type = STRAT_ACTION_TYPE_POINT_TO;
  action3->h.min_duration_ms = 200;
  action3->h.max_duration_ms = 6000;
  action3->speed = 0.4;
  action3->accel = 0.3;
  action3->deccel = 0.3;
  action3->target = dbg_point_to_point3;
  m_action_list[m_n_actions] = (strat_action_t *) action3;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_point_to_t);

  strat_action_goto_astar_t *action4 = (strat_action_goto_astar_t *) curr_act_p;
  action4->h.type = STRAT_ACTION_TYPE_GOTO_ASTAR;
  action4->h.min_duration_ms = 200;
  action4->h.max_duration_ms = 20000;
  action4->speed = 0.3;
  action4->accel = 0.2;
  action4->deccel = 0.2;
  action4->target = dbg_target_point4;
  m_action_list[m_n_actions] = (strat_action_t *) action4;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_goto_astar_t);

  strat_action_point_to_t *action5 = (strat_action_point_to_t *) curr_act_p;
  action5->h.type = STRAT_ACTION_TYPE_POINT_TO;
  action5->h.min_duration_ms = 200;
  action5->h.max_duration_ms = 6000;
  action5->speed = 0.4;
  action5->accel = 0.3;
  action5->deccel = 0.3;
  action5->target = dbg_point_to_point5;
  m_action_list[m_n_actions] = (strat_action_t *) action5;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_point_to_t);

  strat_action_traj_t *action6 = (strat_action_traj_t *) curr_act_p;
  action6->h.type = STRAT_ACTION_TYPE_TRAJ;
  action6->h.min_duration_ms = 200;
  action6->h.max_duration_ms = 20000;
  action6->speed = 0.3;
  action6->accel = 0.2;
  action6->deccel = 0.2;
  action6->nwp = _countof(dbg_traj6);
  memcpy (action6->wp, dbg_traj6, sizeof(dbg_traj6));
  m_action_list[m_n_actions] = (strat_action_t *) action6;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_traj_t);

  strat_action_point_to_t *action7 = (strat_action_point_to_t *) curr_act_p;
  action7->h.type = STRAT_ACTION_TYPE_POINT_TO;
  action7->h.min_duration_ms = 200;
  action7->h.max_duration_ms = 6000;
  action7->speed = 0.4;
  action7->accel = 0.3;
  action7->deccel = 0.3;
  action7->target = dbg_point_to_point7;
  m_action_list[m_n_actions] = (strat_action_t *) action7;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_point_to_t);

  strat_action_goto_astar_t *action8 = (strat_action_goto_astar_t *) curr_act_p;
  action8->h.type = STRAT_ACTION_TYPE_GOTO_ASTAR;
  action8->h.min_duration_ms = 200;
  action8->h.max_duration_ms = 20000;
  action8->speed = 0.3;
  action8->accel = 0.2;
  action8->deccel = 0.2;
  action8->target = dbg_target_point8;
  m_action_list[m_n_actions] = (strat_action_t *) action8;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_goto_astar_t);

  strat_action_point_to_t *action9 = (strat_action_point_to_t *) curr_act_p;
  action9->h.type = STRAT_ACTION_TYPE_POINT_TO;
  action9->h.min_duration_ms = 200;
  action9->h.max_duration_ms = 6000;
  action9->speed = 0.4;
  action9->accel = 0.3;
  action9->deccel = 0.3;
  action9->target = dbg_point_to_point9;
  m_action_list[m_n_actions] = (strat_action_t *) action9;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_point_to_t);

  strat_action_goto_astar_t *action10 = (strat_action_goto_astar_t *)curr_act_p;
  action10->h.type = STRAT_ACTION_TYPE_GOTO_ASTAR;
  action10->h.min_duration_ms = 200;
  action10->h.max_duration_ms = 20000;
  action10->speed = 0.3;
  action10->accel = 0.2;
  action10->deccel = 0.2;
  action10->target = dbg_target_point10;
  m_action_list[m_n_actions] = (strat_action_t *) action10;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_goto_astar_t);

  strat_action_point_to_t *action11 = (strat_action_point_to_t *) curr_act_p;
  action11->h.type = STRAT_ACTION_TYPE_POINT_TO;
  action11->h.min_duration_ms = 200;
  action11->h.max_duration_ms = 6000;
  action11->speed = 0.4;
  action11->accel = 0.3;
  action11->deccel = 0.3;
  action11->target = dbg_point_to_point11;
  m_action_list[m_n_actions] = (strat_action_t *) action11;
  m_n_actions++;
  curr_act_p += sizeof(strat_action_point_to_t);

  return 0;
}

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
  m_path_find_pg.dump_playground_ppm(dump_fname);

  printf ("\n");
}


