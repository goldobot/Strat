#include <string.h>
#include <unistd.h>
#include <math.h>
#include <zmq.h>

#include <fstream>
#include <iostream>
#include <algorithm>

#include "goldo_geometry.hpp"
#include "comm_rplidar.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "robot_state.hpp"
#include "detect/lidar_detect.hpp"
#include "strat/robot_strat_types.hpp"
#include "strat/robot_strat_base.hpp"

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
  memset (m_emergency_action_buf, 0, sizeof(m_emergency_action_buf));
  m_priority = 0;
  m_started = false;
  m_completed = false;
  memset (&m_init_pos_wp, 0, sizeof(m_init_pos_wp));
  memset (&m_init_point_to_wp, 0, sizeof(m_init_point_to_wp));
  m_obstacle_freeze_timeout_ms = 4000;
  m_move_away_dist_mm = 50.0;
  m_current_action_final_wp.x_mm = 1000.0;
  m_current_action_final_wp.y_mm = 0.0;
  m_min_init_goto_duration_ms = 100.0;
  m_max_init_goto_duration_ms = 20000.0; /* FIXME : TODO : heuristics.. */
  m_required_pos_accuracy_mm = 50.0;
  m_required_ang_accuracy_cos = 0.999; /* WARNING : NOT RADIANS! */
}

int StratTask::read_yaml_conf(YAML::Node &yconf)
{
#if 0 /* FIXME : DEBUG */
  if (!yconf["actions"]) 
  {
    printf ("  ERROR : no actions\n");
    return -1;
  }
#endif

  YAML::Node name_node = yconf["task_name"];
  if (name_node) 
  {
    strncpy(m_task_name, name_node.as<std::string>().c_str(), 
            sizeof (m_task_name)-1);
  }
  else
  {
    strncpy(m_task_name, "DebugTask", sizeof (m_task_name)-1);
  }

  YAML::Node init_pos_node = yconf["init_pos"];
  if (init_pos_node) 
  {
    const char *my_str = NULL;
    my_str = init_pos_node[0].as<std::string>().c_str();
    m_init_pos_wp.x_mm = strtof(my_str, NULL);
    my_str = init_pos_node[1].as<std::string>().c_str();
    m_init_pos_wp.y_mm = strtof(my_str, NULL);
  }
  else
  {
    m_init_pos_wp.x_mm = 0.0;
    m_init_pos_wp.y_mm = 0.0;
  }

  YAML::Node init_point_to_node = yconf["init_point_to"];
  if (init_point_to_node) 
  {
    const char *my_str = NULL;
    my_str = init_point_to_node[0].as<std::string>().c_str();
    m_init_point_to_wp.x_mm = strtof(my_str, NULL);
    my_str = init_point_to_node[1].as<std::string>().c_str();
    m_init_point_to_wp.y_mm = strtof(my_str, NULL);
  }
  else
  {
    m_init_point_to_wp.x_mm = 1.0;
    m_init_point_to_wp.y_mm = 0.0;
  }

  YAML::Node obstacle_freeze_timeout_node = yconf["obstacle_freeze_timeout"];
  if (obstacle_freeze_timeout_node) 
  {
    const char *my_str = NULL;
    my_str = obstacle_freeze_timeout_node.as<std::string>().c_str();
    m_obstacle_freeze_timeout_ms = strtoul(my_str, NULL, 10);
  }
  else
  {
    m_obstacle_freeze_timeout_ms = 4000;
  }

  YAML::Node move_away_dist_node = yconf["move_away_dist"];
  if (move_away_dist_node) 
  {
    const char *my_str = NULL;
    my_str = move_away_dist_node.as<std::string>().c_str();
    m_move_away_dist_mm = strtof(my_str, NULL);
  }
  else
  {
    m_move_away_dist_mm = 50.0;
  }

  m_n_actions = yconf["actions"].size();

  m_curr_act_idx = 0;

  memset (m_action_list, 0, sizeof(m_action_list));

  memset (m_action_buf, 0, sizeof(m_action_buf));

  memset (m_emergency_action_buf, 0, sizeof(m_emergency_action_buf));

  m_priority = 0;

  m_started = false;

  m_completed = false;

  unsigned char *curr_act_p = m_action_buf;

  for (int i=0; i<m_n_actions; i++)
  {
    YAML::Node act_node = yconf["actions"][i];
    const char *act_type_str = act_node["type"].as<std::string>().c_str();
    const char *my_str = NULL;

    if (i==0)
    {
      strat_action_t *action = (strat_action_t *) curr_act_p;
      strncpy(action->h.label, "START", sizeof (action->h.label)-1);
    }
    else
    {
      strat_action_t *action = (strat_action_t *) curr_act_p;
      YAML::Node label_node = act_node["label"];
      if (label_node) 
      {
        strncpy(action->h.label, label_node.as<std::string>().c_str(), 
                sizeof (action->h.label)-1);
      }
      else
      {
        memset (action->h.label, 0, sizeof(action->h.label));
      }
    }

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
    else if (strcmp(act_type_str,"STOP")==0)
    {
      strat_action_stop_t *action = (strat_action_stop_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_STOP;
      action->h.min_duration_ms = 0;
      action->h.max_duration_ms = 0;
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
    else if (strcmp(act_type_str,"BRANCH")==0)
    {
      strat_action_branch_t *action = (strat_action_branch_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_BRANCH;
      action->h.min_duration_ms = 0;
      action->h.max_duration_ms = 0;
      my_str = act_node["param_branch"]["condition"].as<std::string>().c_str();
      strncpy(action->condition, my_str, sizeof (action->condition)-1);
      my_str = act_node["param_branch"]["target_if_true"].as<std::string>().c_str();
      strncpy(action->target_if_true, my_str, sizeof (action->target_if_true)-1);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
#if 1 /* FIXME : DEBUG : HACK DEMO2022 */
    else if (strcmp(act_type_str,"FAST_TGT_POSE")==0)
    {
      strat_action_fast_tgt_pose_t *action = (strat_action_fast_tgt_pose_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_FAST_TGT_POSE;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["param_fast_tgt_pose"]["speed"].as<std::string>().c_str();
      action->speed = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_pose"]["accel"].as<std::string>().c_str();
      action->accel = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_pose"]["deccel"].as<std::string>().c_str();
      action->deccel = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_pose"]["target"][0].as<std::string>().c_str();
      action->target.x_mm = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_pose"]["target"][1].as<std::string>().c_str();
      action->target.y_mm = strtof(my_str, NULL);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
    else if (strcmp(act_type_str,"FAST_TGT_OBJECT")==0)
    {
      strat_action_fast_tgt_object_t *action = (strat_action_fast_tgt_object_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_FAST_TGT_OBJECT;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["param_fast_tgt_object"]["speed"].as<std::string>().c_str();
      action->speed = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_object"]["accel"].as<std::string>().c_str();
      action->accel = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_object"]["deccel"].as<std::string>().c_str();
      action->deccel = strtof(my_str, NULL);
      my_str = act_node["param_fast_tgt_object"]["obj_type"].as<std::string>().c_str();
      action->obj_type = strtoul(my_str, NULL, 10);
      my_str = act_node["param_fast_tgt_object"]["obj_attr"].as<std::string>().c_str();
      action->obj_attr = strtoul(my_str, NULL, 10);
      my_str = act_node["param_fast_tgt_object"]["fallback_action"].as<std::string>().c_str();
      strncpy(action->fallback_action, my_str, sizeof (action->fallback_action)-1);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
#endif
#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    else if (strcmp(act_type_str,"CRIDF2021")==0)
    {
      strat_action_cridf2021_t *action = (strat_action_cridf2021_t *) curr_act_p;
      action->h.type = STRAT_ACTION_TYPE_CRIDF2021;
      my_str = act_node["min_duration_ms"].as<std::string>().c_str();
      action->h.min_duration_ms = strtoul(my_str, NULL, 10);
      my_str = act_node["max_duration_ms"].as<std::string>().c_str();
      action->h.max_duration_ms = strtoul(my_str, NULL, 10);
      m_action_list[i] = (strat_action_t *) action;
      curr_act_p += sizeof(*action);
    }
#endif
    else
    {
      printf ("  ERROR : unknown action (index=%d, act_type_str=%s)\n", i, act_type_str);
    }

  } /* for (int i=0; i<m_n_actions; i++) */

  return 0;
}

int StratTask::get_action_idx_with_label(char *label)
{
  int idx = -1;

  for (int i=0; i<m_n_actions; i++)
  {
    strat_action_t *action = m_action_list[i];
    if (strcmp(label,action->h.label)==0)
    {
      idx = i;
      break;
    }
  }

  return idx;
}

/******************************************************************************/
/**  Playground  **************************************************************/
/******************************************************************************/

StratPlayground::StratPlayground()
{
  memset (m_stat_pattern, 0, sizeof(m_stat_pattern));
  memset (m_mob_pattern, 0, sizeof(m_mob_pattern));
  memset (m_free_pattern, 0, sizeof(m_free_pattern));
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
  /* FIXME : TODO : don't use the PATH code for the free zone! */
  init_pattern(m_free_pattern, M_FREE_SZ_CM, M_FREE_R_CM, PATH);

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
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM, true);
  }

  x = X_MAX_CM-1;
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM, true);
  }

  y = Y_MIN_CM;
  for (x=X_MIN_CM; x<X_MAX_CM; x++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM, true);
  }

  y = Y_MAX_CM-1;
  for (x=X_MIN_CM; x<X_MAX_CM; x++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM, true);
  }

  /* static obstacles 2020 */
  //put_stat_rect_obst(1850,  -610, 2000,  -590); /* 2020 left rocks   */
  //put_stat_rect_obst(1850,   610, 2000,   590); /* 2020 right rocks  */
  //put_stat_rect_obst(1700,   -10, 2000,    10); /* 2020 center rocks */

  //put_stat_rect_obst(1700,  -450, 2000,  -150); /* 2020 left little harbor  */
  //put_stat_rect_obst(1700,   450, 2000,   150); /* 2020 right little harbor */
  //put_stat_rect_obst( 500, -1100, 1100, -1500); /* 2020 left big harbor     */
  //put_stat_rect_obst( 500,  1100, 1100,  1500); /* 2020 right big harbor    */

  /* FIXME : TODO : make these zones (static obstacles) configurable */
  /* static obstacles 2022 */
  put_stat_triangle_obst( 2000, -1500,  1490, -1500,  2000,  -990); /* abri chantier gauche */
  put_stat_rect_obst( 1170, -1500,  1330, -1400); /* distributeur gauche */
  put_stat_rect_obst(  400, -1500,  1000, -1100); /* zone depart gauche  */
  put_stat_rect_obst(    0, -1050,   100,  -330); /* galerie gauche      */
  put_stat_rect_obst(    0,  -230,   100,   -70); /* distributeur gauche */
  put_stat_rect_obst(    0,   -10,   300,    10); /* separateur central  */
  put_stat_rect_obst(    0,   230,   100,    70); /* distributeur droit  */
  put_stat_rect_obst(    0,  1050,   100,   330); /* galerie droite      */
  put_stat_rect_obst(  400,  1500,  1000,  1100); /* zone depart droite  */
  put_stat_rect_obst( 1170,  1500,  1330,  1400); /* distributeur droit  */
  put_stat_triangle_obst( 2000,  1500,  1490,  1500,  2000,   990); /* abri chantier droit  */

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
                                  int _patt_sz_cm, bool is_obst)
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
        if (is_obst)
        {
          if (m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)]==NO_OBST)
            m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)] = 
              _patt[(yy)*(_patt_sz_cm) + xx];
        }
        else
        {
          if ((m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)]!=NO_OBST) &&
              (_patt[(yy)*(_patt_sz_cm) + xx]!=NO_OBST))
            m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)] = NO_OBST;
            
        }
      }
    }
  }
}

void StratPlayground::put_stat_rect_obst(int x_min_mm, int y_min_mm,
                                         int x_max_mm, int y_max_mm)
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
      put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM, true);
    }
  }
}

void StratPlayground::put_stat_triangle_obst(int x1_mm, int y1_mm, int x2_mm, int y2_mm, int x3_mm, int y3_mm)
{
  int x, y;
  int x_min_mm = std::min({x1_mm,x2_mm,x3_mm});
  int y_min_mm = std::min({y1_mm,y2_mm,y3_mm});
  int x_max_mm = std::max({x1_mm,x2_mm,x3_mm});
  int y_max_mm = std::max({y1_mm,y2_mm,y3_mm});
  int x_min_cm = x_min_mm/10;
  int y_min_cm = y_min_mm/10;
  int x_max_cm = x_max_mm/10;
  int y_max_cm = y_max_mm/10;

  goldo_vec_2d_t v1 = {(float)x1_mm, (float)y1_mm};
  goldo_vec_2d_t v2 = {(float)x2_mm, (float)y2_mm};
  goldo_vec_2d_t v3 = {(float)x3_mm, (float)y3_mm};

  for (y=y_min_cm; y<y_max_cm; y++)
  {
    for (x=x_min_cm; x<x_max_cm; x++)
    {
      goldo_vec_2d_t p_mm = {10.0*x, 10.0*y};
      if (goldo_point_in_triangle(p_mm,v1,v2,v3))
      {
        m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
        put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM, true);
      }
    }
  }
}

void StratPlayground::put_mob_point_obst(int x_mm, int y_mm)
{
  int x = x_mm/10;
  int y = y_mm/10;

  put_pattern(x, y, m_mob_pattern, M_PATT_SZ_CM, true);
  m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = M_OBST;

#if 0 /* FIXME : DEBUG : HACK coupe */
  if ((x_mm>750) && (y_mm>-500) && (y_mm<500))
  {
    hack_coupe(x_mm, y_mm);
  }
#endif
}

#if 1 /* FIXME : DEBUG : HACK coupe */
void StratPlayground::hack_coupe(int x_mm, int y_mm)
{
  int x = x_mm/10;
  int y = y_mm/10;

  int yy = y;
  for (int xx=x; xx<X_MAX_CM; xx++)
  {
    m_playground[(yy+Y_OFFSET_CM)*(X_SZ_CM) + xx] = S_EXCL;
  }
}
#endif

void StratPlayground::put_free_zone(int x_mm, int y_mm)
{
  int x = x_mm/10;
  int y = y_mm/10;

  put_pattern(x, y, m_free_pattern, M_FREE_SZ_CM, false);
  m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = M_OBST;
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
/**  DEBUG  *******************************************************************/
/******************************************************************************/

void StratTask::dbg_dump_task()
{
  int i,j;

  printf ("\n");
  printf ("dbg_task:\n");
  printf ("  task_name: %s\n", m_task_name);
  printf ("  init_pos:\n");
  printf ("    [%8.1f,%8.1f]\n", 
          m_init_pos_wp.x_mm, m_init_pos_wp.y_mm);
  printf ("  init_point_to:\n");
  printf ("    [%8.1f,%8.1f]\n", 
          m_init_point_to_wp.x_mm, m_init_point_to_wp.y_mm);
  printf ("  obstacle_freeze_timeout: %d\n", m_obstacle_freeze_timeout_ms);
  printf ("  move_away_dist: %8.1f\n", m_move_away_dist_mm);
  printf ("  curr_act_idx: %d\n", m_curr_act_idx);
  printf ("  n_actions: %d\n", m_n_actions);
  printf ("  actions:\n");
  for (i=0; i<m_n_actions; i++)
  {
    strat_action_t *act = m_action_list[i];
    printf ("  - # action %d:\n", i);
    printf ("    label: %s\n", act->h.label);
    printf ("    min_duration_ms: %d\n", act->h.min_duration_ms);
    printf ("    max_duration_ms: %d\n", act->h.max_duration_ms);
    switch (act->h.type) {
    case STRAT_ACTION_TYPE_NONE:
      printf ("    type: NONE\n");
      break;
    case STRAT_ACTION_TYPE_WAIT:
      printf ("    type: WAIT\n");
      break;
    case STRAT_ACTION_TYPE_STOP:
      printf ("    type: STOP\n");
      break;
    case STRAT_ACTION_TYPE_TRAJ:
    {
      strat_action_traj_t *act_traj = (strat_action_traj_t *)act;
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
    }
    case STRAT_ACTION_TYPE_POINT_TO:
    {
      strat_action_point_to_t *act_point = (strat_action_point_to_t *)act;
      printf ("    type: POINT_TO\n");
      printf ("    param_point:\n");
      printf ("      speed  : %f\n", act_point->speed);
      printf ("      accel  : %f\n", act_point->accel);
      printf ("      deccel : %f\n", act_point->deccel);
      printf ("      target:\n");
      printf ("        [%8.1f,%8.1f]\n", 
              act_point->target.x_mm, act_point->target.y_mm);
      break;
    }
    case STRAT_ACTION_TYPE_NUCLEO_SEQ:
    {
      strat_action_nucleo_seq_t *act_nuc = (strat_action_nucleo_seq_t *)act;
      printf ("    type: NUCLEO_SEQ\n");
      printf ("    param_nucleo_seq:\n");
      printf ("      nucleo_seq_id: %d\n", act_nuc->nucleo_seq_id);
      break;
    }
    case STRAT_ACTION_TYPE_GOTO_ASTAR:
    {
      strat_action_goto_astar_t *act_astar = (strat_action_goto_astar_t *)act;
      printf ("    type: GOTO_ASTAR\n");
      printf ("    param_goto_astar:\n");
      printf ("      speed  : %f\n", act_astar->speed);
      printf ("      accel  : %f\n", act_astar->accel);
      printf ("      deccel : %f\n", act_astar->deccel);
      printf ("      target:\n");
      printf ("        [%8.1f,%8.1f]\n", 
              act_astar->target.x_mm, act_astar->target.y_mm);
      break;
    }
    case STRAT_ACTION_TYPE_BRANCH:
    {
      strat_action_branch_t *act_branch = (strat_action_branch_t *)act;
      printf ("    type: BRANCH\n");
      printf ("    param_branch:\n");
      printf ("      condition       : %s\n", act_branch->condition);
      printf ("      target_if_true  : %s\n", act_branch->target_if_true);
      break;
    }
#if 1 /* FIXME : DEBUG : HACK DEMO2022 */
    case STRAT_ACTION_TYPE_FAST_TGT_POSE:
    {
      strat_action_fast_tgt_pose_t *act_fast_tgt_pose = (strat_action_fast_tgt_pose_t *)act;
      printf ("    type: FAST_TGT_POSE\n");
      printf ("    param_fast_tgt_pose:\n");
      printf ("      speed  : %f\n", act_fast_tgt_pose->speed);
      printf ("      accel  : %f\n", act_fast_tgt_pose->accel);
      printf ("      deccel : %f\n", act_fast_tgt_pose->deccel);
      printf ("      target:\n");
      printf ("        [%8.1f,%8.1f]\n", 
              act_fast_tgt_pose->target.x_mm, act_fast_tgt_pose->target.y_mm);
      break;
    }
    case STRAT_ACTION_TYPE_FAST_TGT_OBJECT:
    {
      strat_action_fast_tgt_object_t *act_fast_tgt_object = (strat_action_fast_tgt_object_t *)act;
      printf ("    type: FAST_TGT_OBJECT\n");
      printf ("    param_fast_tgt_object:\n");
      printf ("      speed  : %f\n", act_fast_tgt_object->speed);
      printf ("      accel  : %f\n", act_fast_tgt_object->accel);
      printf ("      deccel : %f\n", act_fast_tgt_object->deccel);
      printf ("      obj_type : %d\n", act_fast_tgt_object->obj_type);
      printf ("      obj_attr : %d\n", act_fast_tgt_object->obj_attr);
      break;
    }
#endif
#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
    case STRAT_ACTION_TYPE_CRIDF2021:
      printf ("    type: CRIDF2021\n");
      break;
#endif
    default:
      printf ("    UNKNOWN type!(%d)\n", act->h.type);
    }
  }
}




