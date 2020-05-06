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

int StratTask::read_yaml_conf(YAML::Node &yconf)
{
  if (!yconf["actions"]) 
  {
    printf ("  ERROR : no actions\n");
    return -1;
  }

  YAML::Node name_node = yconf["task_name"];
  if (name_node) 
  {
    strncpy(m_task_name, name_node.as<std::string>().c_str(), 
            sizeof (m_task_name));
  }
  else
  {
    strncpy(m_task_name, "DebugTask", sizeof (m_task_name));
  }

  m_n_actions = yconf["actions"].size();

  m_curr_act_idx = 0;

  memset (m_action_list, 0, sizeof(m_action_list));

  memset (m_action_buf, 0, sizeof(m_action_buf));

  m_priority = 0;

  m_started = false;

  m_completed = false;

  unsigned char *curr_act_p = m_action_buf;

  for (int i=0; i<m_n_actions; i++)
  {
    YAML::Node act_node = yconf["actions"][i];
    const char *act_type_str = act_node["type"].as<std::string>().c_str();
    const char *my_str = NULL;

    if (strcmp(act_type_str,"WAIT")==0)
    {
      strat_action_wait_t *action = (strat_action_wait_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_WAIT;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
    else if (strcmp(act_type_str,"TRAJ")==0)
    {
      strat_action_traj_t *action = (strat_action_traj_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_TRAJ;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["param_traj"]["speed"].as<std::string>().c_str();
      action->speed = strtof(my_str, NULL);
      my_str = act_node["param_traj"]["accel"].as<std::string>().c_str();
      action->accel = strtof(my_str, NULL);
      my_str = act_node["param_traj"]["deccel"].as<std::string>().c_str();
      action->deccel = strtof(my_str, NULL);
      YAML::Node wp_node = act_node["param_traj"]["wp"];
      action->nwp = wp_node.size();
      for (int j=0; j<action->nwp; j++)
      {
        my_str = wp_node[j][0].as<std::string>().c_str();
        action->wp[j].x_mm = strtof(my_str, NULL);
        my_str = wp_node[j][1].as<std::string>().c_str();
        action->wp[j].y_mm = strtof(my_str, NULL);
      }
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
    else if (strcmp(act_type_str,"POINT_TO")==0)
    {
      strat_action_point_to_t *action = (strat_action_point_to_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_POINT_TO;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["param_point"]["speed"].as<std::string>().c_str();
      action->speed = strtof(my_str, NULL);
      my_str = act_node["param_point"]["accel"].as<std::string>().c_str();
      action->accel = strtof(my_str, NULL);
      my_str = act_node["param_point"]["deccel"].as<std::string>().c_str();
      action->deccel = strtof(my_str, NULL);
      my_str = act_node["param_point"]["target"][0].as<std::string>().c_str();
      action->target.x_mm = strtof(my_str, NULL);
      my_str = act_node["param_point"]["target"][1].as<std::string>().c_str();
      action->target.y_mm = strtof(my_str, NULL);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
    else if (strcmp(act_type_str,"NUCLEO_SEQ")==0)
    {
      strat_action_nucleo_seq_t *action = (strat_action_nucleo_seq_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_NUCLEO_SEQ;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["param_nucleo_seq"]["id"].as<std::string>().c_str();
      action->nucleo_seq_id = strtoul(my_str, NULL, 10);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
    else if (strcmp(act_type_str,"GOTO_ASTAR")==0)
    {
      strat_action_goto_astar_t *action = (strat_action_goto_astar_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_GOTO_ASTAR;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["param_goto_astar"]["speed"].as<std::string>().c_str();
      action->speed = strtof(my_str, NULL);
      my_str = act_node["param_goto_astar"]["accel"].as<std::string>().c_str();
      action->accel = strtof(my_str, NULL);
      my_str = act_node["param_goto_astar"]["deccel"].as<std::string>().c_str();
      action->deccel = strtof(my_str, NULL);
      my_str = act_node["param_goto_astar"]["target"][0].as<std::string>().c_str();
      action->target.x_mm = strtof(my_str, NULL);
      my_str = act_node["param_goto_astar"]["target"][1].as<std::string>().c_str();
      action->target.y_mm = strtof(my_str, NULL);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
    else
    {
      printf ("  ERROR : unknown action\n");
    }

  } /* for (int i=0; i<m_n_actions; i++) */

  return 0;
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
  m_ppm_sz = 0;
  memset (m_ppm_buff, 0, sizeof(m_ppm_buff));
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

  /* ppm image for debug */
  m_ppm_sz = 0;
  memset (m_ppm_buff, 0, sizeof(m_ppm_buff));
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

void StratPlayground::create_playground_ppm()
{
  unsigned char *p=m_ppm_buff;
  const int dimH = 300, dimV = 200;
  int i, j;

  m_ppm_sz = 0;
  memset (m_ppm_buff, 0, sizeof(m_ppm_buff));

  int head_sz = sprintf((char *)p, "P6\n%d %d\n255\n", dimH, dimV);
  p += head_sz;
  m_ppm_sz += head_sz;

  for (j = 0; j < dimV; ++j)
  {
    for (i = 0; i < dimH; ++i)
    {
      unsigned char color[3];
      color[0] = 255;  /* red   */
      color[1] = 255;  /* green */
      color[2] = 255;  /* blue  */

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

      p[0] = color[0];
      p[1] = color[1];
      p[2] = color[2];
      p += 3;
      m_ppm_sz += 3;
    }
  }

}

void StratPlayground::dump_playground_ppm(char *ppm_fname)
{
  FILE *fp = fopen(ppm_fname, "wb"); /* b - binary mode */
  (void) fwrite(m_ppm_buff, 1, m_ppm_sz, fp);
  (void) fclose(fp);
}

void StratPlayground::send_playground_ppm()
{
  unsigned short my_message_type = 2051;
  unsigned char compress_flag = 0;

  if (m_ppm_sz==0) return;

  CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
  CommZmq::instance().send((const char*)(&compress_flag), 1, ZMQ_SNDMORE);
  CommZmq::instance().send((const char*)(m_ppm_buff), m_ppm_sz, 0);
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
  if (m_task_dbg.read_yaml_conf(dbg_node)!=0)
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
  usleep (100000);

  /* FIXME : DEBUG */
  m_dbg_step_by_step = true;

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
      m_task_dbg.m_curr_act_idx = 0;

      if ((!RobotState::instance().tirette_present()) || m_start_match_sig)
      {
        printf ("\n DEBUG : GO!..\n\n");

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
          sprintf(m_dbg_fname,"dump_astar_act%d.ppm",m_task_dbg.m_curr_act_idx);
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

void StratTask::dbg_dump_task()
{
  int i,j;
  strat_action_traj_t *act_traj = NULL;
  strat_action_point_to_t *act_point = NULL;
  strat_action_nucleo_seq_t *act_nuc = NULL;
  strat_action_goto_astar_t *act_astar = NULL;

  printf ("\n");
  printf ("dbg_task:\n");
  printf ("  task_name: %s\n", m_task_name);
  printf ("  curr_act_idx: %d\n", m_curr_act_idx);
  printf ("  n_actions: %d\n", m_n_actions);
  printf ("  actions:\n");
  for (i=0; i<m_n_actions; i++)
  {
    strat_action_t *act = m_action_list[i];
    printf ("  - # action %d:\n", i);
    printf ("    min_duration_ms: %d\n", act->h.min_duration_ms);
    printf ("    max_duration_ms: %d\n", act->h.max_duration_ms);
    switch (act->h.type) {
    case STRAT_ACTION_TYPE_NONE:
      printf ("    type: NONE\n");
      break;
    case STRAT_ACTION_TYPE_WAIT:
      printf ("    type: WAIT\n");
      break;
    case STRAT_ACTION_TYPE_TRAJ:
      act_traj = (strat_action_traj_t *)act;
      printf ("    type: TRAJ\n");
      printf ("    param_traj:\n");
      printf ("      speed  : %f\n", act_traj->speed);
      printf ("      accel  : %f\n", act_traj->accel);
      printf ("      deccel : %f\n", act_traj->deccel);
      printf ("      nwp    : %d\n", act_traj->nwp);
      printf ("      wp:\n");
      for (j=0; j<act_traj->nwp; j++)
      {
        printf ("        - [%8.1f,%8.1f]\n", 
                act_traj->wp[j].x_mm, act_traj->wp[j].y_mm);
      }
      break;
    case STRAT_ACTION_TYPE_POINT_TO:
      act_point = (strat_action_point_to_t *)act;
      printf ("    type: POINT_TO\n");
      printf ("    param_point:\n");
      printf ("      speed  : %f\n", act_point->speed);
      printf ("      accel  : %f\n", act_point->accel);
      printf ("      deccel : %f\n", act_point->deccel);
      printf ("      target:\n");
      printf ("        [%8.1f,%8.1f]\n", 
              act_point->target.x_mm, act_point->target.y_mm);
      break;
    case STRAT_ACTION_TYPE_NUCLEO_SEQ:
      act_nuc = (strat_action_nucleo_seq_t *)act;
      printf ("    type: NUCLEO_SEQ\n");
      printf ("    param_nucleo_seq:\n");
      printf ("      nucleo_seq_id: %d\n", act_nuc->nucleo_seq_id);
      break;
    case STRAT_ACTION_TYPE_GOTO_ASTAR:
      act_astar = (strat_action_goto_astar_t *)act;
      printf ("    type: GOTO_ASTAR\n");
      printf ("    param_goto_astar:\n");
      printf ("      speed  : %f\n", act_astar->speed);
      printf ("      accel  : %f\n", act_astar->accel);
      printf ("      deccel : %f\n", act_astar->deccel);
      printf ("      target:\n");
      printf ("        [%8.1f,%8.1f]\n", 
              act_astar->target.x_mm, act_astar->target.y_mm);
      break;
    default:
      printf ("    UNKNOWN type!(%d)\n", act->h.type);
    }
  }
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
  m_path_find_pg.create_playground_ppm();
  m_path_find_pg.dump_playground_ppm(dump_fname);

  printf ("\n");
}

void RobotStrat::dbg_dump()
{
  m_task_dbg.dbg_dump_task();
}



