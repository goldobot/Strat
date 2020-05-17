#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>


#define ROBOT_SIM 1
//#define SIM_DEBUG 1

#include "robot_state.hpp"
#include "sim/virtual_robot.hpp"

using namespace goldobot;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

MyselfVirtualRobotType VirtualRobots::s_myself;

PartnerVirtualRobotType VirtualRobots::s_partner;

Adversary1VirtualRobotType VirtualRobots::s_adversary1;

Adversary2VirtualRobotType VirtualRobots::s_adversary2;



VirtualRobot::VirtualRobot()
{
  m_enabled = false;

  m_automatic = true;

  memset(&m_sv, 0, sizeof(m_sv));

  m_gpio = GPIO_START_MASK;

  m_default_strat = new StratTask();
  if (!m_default_strat)
  {
    m_default_strat = NULL;
    printf ("ERROR : VirtualRobot::VirtualRobot() : cannot alloc memory\n");
  }

  m_me = new sim_motion_element_t[MOTION_ELEM_LIST_SZ];
  if (m_me)
  {
    memset(m_me, 0, MOTION_ELEM_LIST_SZ*sizeof(sim_motion_element_t));
  }
  else
  {
    m_me = NULL;
    printf ("ERROR : VirtualRobot::VirtualRobot() : cannot alloc memory\n");
  }
  m_me_idx = -1;
  m_me_cnt = 0;


  memset(&m_recv_buf, 0, sizeof(m_recv_buf));

  /* FIXME : TODO : geometric model of the robot (with conf section) */
  m_prop_semi_axis = 0.1;

  /* FIXME : TODO */
  m_dbg_duration = 0.0;
}

int VirtualRobot::read_yaml_conf(YAML::Node &yconf)
{
  if(m_default_strat->read_yaml_conf(yconf)==0) {
    sim_vec_2d_t orig_vec;
    sim_vec_2d_t target_vec;
    orig_vec.x = m_default_strat->m_init_pos_wp.x_mm*0.001;
    orig_vec.y = m_default_strat->m_init_pos_wp.y_mm*0.001;
    target_vec.x = m_default_strat->m_init_point_to_wp.x_mm*0.001;
    target_vec.y = m_default_strat->m_init_point_to_wp.y_mm*0.001;
    m_sv.p = orig_vec;
    m_sv.theta = sim_compute_theta(orig_vec, target_vec);
#ifdef SIM_DEBUG
    printf ("\n");
    printf ("DEBUG : VirtualRobot::read_yaml_conf() : m_sv.p.x=%f mm\n",
            m_sv.p.x*1000.0);
    printf ("DEBUG : VirtualRobot::read_yaml_conf() : m_sv.p.y=%f mm\n",
            m_sv.p.y*1000.0);
    printf ("DEBUG : VirtualRobot::read_yaml_conf() : m_sv.theta=%f°\n",
            (m_sv.theta*180.0/M_PI));
#endif /* SIM_DEBUG */
  }
  else
  {
    return -1;
  }

  return 0;
}

void VirtualRobot::sim_update(double t_inc)
{
  double old_v_x     = m_sv.v.x;
  double old_v_y     = m_sv.v.y;
  double old_v_theta = m_sv.v_theta;

  if ((m_me!=NULL) && (m_me_idx!=-1) && (m_me_idx!=m_me_cnt))
  {
    if (m_me[m_me_cnt].type == SM_ELEM_DYN)
    {
      double a_x     = m_me[m_me_idx].u.dyn.lin_accel * cos(m_sv.theta);
      double a_y     = m_me[m_me_idx].u.dyn.lin_accel * sin(m_sv.theta);
      double a_theta = m_me[m_me_idx].u.dyn.ang_accel;

      if ((m_me[m_me_idx].u.dyn.duration-t_inc)<0.0) 
      {
        t_inc = m_me[m_me_idx].u.dyn.duration;
        m_me[m_me_idx].u.dyn.duration = 0.0;

#ifdef SIM_DEBUG
        printf ("\n");
        printf ("DEBUG : VirtualRobot::sim_update() : m_dbg_duration=%f\n",
                m_dbg_duration);
        printf ("DEBUG : VirtualRobot::sim_update() : m_sv.p.x=%f mm\n",
                m_sv.p.x*1000.0);
        printf ("DEBUG : VirtualRobot::sim_update() : m_sv.p.y=%f mm\n",
                m_sv.p.y*1000.0);
        printf ("DEBUG : VirtualRobot::sim_update() : m_sv.v.x=%f mm/s\n",
                m_sv.v.x*1000.0);
        printf ("DEBUG : VirtualRobot::sim_update() : m_sv.v.y=%f mm/s\n",
                m_sv.v.y*1000.0);
        printf ("DEBUG : VirtualRobot::sim_update() : a_x=%f mm/s2\n",
                a_x*1000.0);
        printf ("DEBUG : VirtualRobot::sim_update() : a_y=%f mm/s2\n",
                a_y*1000.0);
        printf ("DEBUG : VirtualRobot::sim_update() : m_sv.theta=%f°\n",
                (m_sv.theta*180.0/M_PI));
        printf ("DEBUG : VirtualRobot::sim_update() : m_sv.v_theta=%f(%f°)\n",
                m_sv.v_theta, (m_sv.v_theta*180.0/M_PI));
        printf ("DEBUG : VirtualRobot::sim_update() : a_theta=%f(%f°)\n",
                a_theta, (a_theta*180.0/M_PI));
#endif /* SIM_DEBUG */
        m_me_idx++;
        if (m_me_idx==m_me_cnt) 
        {
          m_gpio = m_gpio&(~FLAG_PROPULSION_BUSY_MASK);
#ifdef SIM_DEBUG
          printf ("DEBUG : VirtualRobot::sim_update() : end actions\n");
#endif /* SIM_DEBUG */
          m_dbg_duration = 0.0;
        }
      }
      else
      {
        m_me[m_me_idx].u.dyn.duration -= t_inc;
      }

      m_dbg_duration += t_inc;

      m_sv.v.x     += a_x*t_inc;
      m_sv.v.y     += a_y*t_inc;
      m_sv.v_theta += a_theta*t_inc;

    }
    else
    {
      /* FIXME : TODO */
      m_me_idx++;
      if (m_me_idx==m_me_cnt) 
      {
        m_gpio = m_gpio&(~FLAG_PROPULSION_BUSY_MASK);
#ifdef SIM_DEBUG
        printf ("DEBUG : VirtualRobot::sim_update() : no actions\n");
        printf ("DEBUG : VirtualRobot::sim_update() : m_dbg_duration=%f\n",
                m_dbg_duration);
#endif /* SIM_DEBUG */
        m_dbg_duration = 0.0;
      }
    }
  }

  m_sv.p.x   += old_v_x*t_inc;
  m_sv.p.y   += old_v_y*t_inc;
  m_sv.theta += old_v_theta*t_inc;
  m_sv.theta = sim_normalize_angle(m_sv.theta);

}

void VirtualRobot::sim_receive(const unsigned char *msg_buf, size_t msg_len)
{
  if (!m_enabled) 
  {
    printf ("  ERROR : VirtualRobot::sim_receive() : not enabled\n");
    return;
  }

  if (m_automatic) 
  {
    printf ("  ERROR : VirtualRobot::sim_receive() : not controlable\n");
    return;
  }

  if (m_me==NULL) 
  {
    printf ("  ERROR : VirtualRobot::sim_receive() : "
            "cannot execute order (not enough memory?)\n");
    return;
  }

  if (msg_len>RECV_BUF_SZ) 
  {
    printf ("  ERROR : VirtualRobot::sim_receive() : msg_len>RECV_BUF_SZ\n");
    return;
  }

  memcpy(m_recv_buf, msg_buf, msg_len);

  unsigned char *_pc = m_recv_buf;
  unsigned short int cmd_traj_code;
  int field_len = 0;

  field_len = sizeof(unsigned short int);
  memcpy ((unsigned char *)&cmd_traj_code, _pc, field_len);
  /* FIXME : TODO : use defines.. */
  switch (cmd_traj_code) {
  case 0x0055: /*DbgPropulsionExecuteTrajectory*/
    on_cmd_execute_trajectory (m_recv_buf+2, msg_len-2);
    break;
  case 0x0058: /*DbgPropulsionExecutePointTo*/
    on_cmd_execute_point_to (m_recv_buf+2, msg_len-2);
    break;
  case 0x0053: /*DbgPropulsionSetPose*/
    on_cmd_set_pose (m_recv_buf+2, msg_len-2);
    break;
  case 0x002b: /*MainSequenceStartSequence*/
    on_cmd_start_sequence (m_recv_buf+2, msg_len-2);
    break;
  case 0x0063: /*PropulsionClearError*/
    on_cmd_propulsion_clear_error (m_recv_buf+2, msg_len-2);
    break;
  }

}

void VirtualRobot::sim_brutal_stop()
{
  m_me_idx = -1;
  m_me_cnt = 0;
  memset(m_me, 0, MOTION_ELEM_LIST_SZ*sizeof(sim_motion_element_t));
  m_sv.v.x = 0.0;
  m_sv.v.y = 0.0;
  m_sv.v_theta = 0.0;
  m_gpio = m_gpio&(~FLAG_PROPULSION_BUSY_MASK);
}

void VirtualRobot::sim_compute_motion_times(
  double L, double V, 
  double A, double D, 
  double &t_a, double &t_i, double &t_d)
{
  t_a = V/A;
  t_d = V/D;
  t_i = (L-A*t_a*t_a/2-D*t_d*t_d/2)/V;
  if (t_i<=0.0)
  {
    t_i = 0.0;
    t_a = sqrt((2.0*L*D)/(A*(A+D)));
    t_d = t_a*A/D;
  }
}

void VirtualRobot::create_rotation_me(sim_vec_2d_t &orig, double orig_theta, 
                                      sim_vec_2d_t &target, 
                                      float speed, float accel, float deccel)
{
#ifdef SIM_DEBUG
  printf ("DEBUG : create_rotation_me() : orig=[%f,%f]\n", orig.x, orig.y);
  printf ("DEBUG : create_rotation_me() : target=[%f,%f]\n",target.x,target.y);
#endif /* SIM_DEBUG */

  double new_theta = sim_compute_theta(orig, target);
  double d_theta = sim_normalize_angle(new_theta - orig_theta);

#ifdef SIM_DEBUG
  printf ("DEBUG : create_rotation_me() : orig_theta=%f\n", orig_theta);
  printf ("DEBUG : create_rotation_me() : new_theta=%f\n", new_theta);
  printf ("DEBUG : create_rotation_me() : d_theta=%f(%f°)\n", 
          d_theta, d_theta*180.0/M_PI);
#endif /* SIM_DEBUG */

  speed  = speed/m_prop_semi_axis;
  accel  = accel/m_prop_semi_axis;
  deccel = deccel/m_prop_semi_axis;

  double t_a;
  double t_i;
  double t_d;
  sim_compute_motion_times(fabs(d_theta), speed, accel, deccel, t_a, t_i, t_d);
#ifdef SIM_DEBUG
  printf ("DEBUG : create_rotation_me() : t_a=%f\n", t_a);
  printf ("DEBUG : create_rotation_me() : t_i=%f\n", t_i);
  printf ("DEBUG : create_rotation_me() : t_d=%f\n", t_d);
#endif /* SIM_DEBUG */

  if (d_theta<0)
  {
    speed  = -speed;
    accel  = -accel;
    deccel = -deccel;
  }

  m_me[m_me_cnt].type = SM_ELEM_DYN;
  m_me[m_me_cnt].u.dyn.duration  = t_a;
  m_me[m_me_cnt].u.dyn.lin_accel = 0.0;
  m_me[m_me_cnt].u.dyn.ang_accel = accel;
  m_me_cnt++;

  if (t_i>0.0)
  {
    m_me[m_me_cnt].type = SM_ELEM_DYN;
    m_me[m_me_cnt].u.dyn.duration  = t_i;
    m_me[m_me_cnt].u.dyn.lin_accel = 0.0;
    m_me[m_me_cnt].u.dyn.ang_accel = 0.0;
    m_me_cnt++;
  }

  m_me[m_me_cnt].type = SM_ELEM_DYN;
  m_me[m_me_cnt].u.dyn.duration  = t_d;
  m_me[m_me_cnt].u.dyn.lin_accel = 0.0;
  m_me[m_me_cnt].u.dyn.ang_accel = -deccel;
  m_me_cnt++;
}

void VirtualRobot::create_translation_me(sim_vec_2d_t &orig, bool forward,
                                         sim_vec_2d_t &target, 
                                         float speed, float accel, float deccel)
{
  double dist = sim_dist(target, orig);

  double t_a;
  double t_i;
  double t_d;
  sim_compute_motion_times(dist, speed, accel, deccel, t_a, t_i, t_d);
#ifdef SIM_DEBUG
  printf ("DEBUG : create_translation_me() : t_a=%f\n", t_a);
  printf ("DEBUG : create_translation_me() : t_i=%f\n", t_i);
  printf ("DEBUG : create_translation_me() : t_d=%f\n", t_d);
#endif /* SIM_DEBUG */

  if (!forward)
  {
    speed  = -speed;
    accel  = -accel;
    deccel = -deccel;
  }

  m_me[m_me_cnt].type = SM_ELEM_DYN;
  m_me[m_me_cnt].u.dyn.duration  = t_a;
  m_me[m_me_cnt].u.dyn.lin_accel = accel;
  m_me[m_me_cnt].u.dyn.ang_accel = 0.0;
  m_me_cnt++;

  if (t_i>0.0)
  {
    m_me[m_me_cnt].type = SM_ELEM_DYN;
    m_me[m_me_cnt].u.dyn.duration  = t_i;
    m_me[m_me_cnt].u.dyn.lin_accel = 0.0;
    m_me[m_me_cnt].u.dyn.ang_accel = 0.0;
    m_me_cnt++;
  }

  m_me[m_me_cnt].type = SM_ELEM_DYN;
  m_me[m_me_cnt].u.dyn.duration  = t_d;
  m_me[m_me_cnt].u.dyn.lin_accel = -deccel;
  m_me[m_me_cnt].u.dyn.ang_accel = 0.0;
  m_me_cnt++;
}


void VirtualRobot::on_cmd_execute_trajectory(unsigned char *msg_buf, 
                                             size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float speed;
  float accel;
  float deccel;
  int nwp;
  strat_way_point_t *wp;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&speed, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&accel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&deccel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  nwp = msg_len/(2*sizeof(float));

  wp = new strat_way_point_t[nwp];

  for (int i=0; i<nwp; i++) 
  {
    float my_x;
    float my_y;

    field_len = sizeof(float);
    memcpy ((unsigned char *)&(my_x), _pc, field_len);
    _pc += field_len;
    msg_len -= field_len;

    field_len = sizeof(float);
    memcpy ((unsigned char *)&(my_y), _pc, field_len);
    _pc += field_len;
    msg_len -= field_len;

    wp[i].x_mm = my_x*1000.0;
    wp[i].y_mm = my_y*1000.0;
  }

  if (nwp<2)
  {
    delete wp;
    return;
  }

#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobot::on_cmd_execute_trajectory():\n");
  printf ("  speed = %f\n", speed);
  printf ("  accel = %f\n", accel);
  printf ("  deccel = %f\n", deccel);
  printf ("  nwp = %d\n", nwp);
  printf ("  wp:\n");
  for (int i=0; i<nwp; i++) 
  {
    printf ("    [%f, %f]\n", wp[i].x_mm, wp[i].y_mm);
  }
  printf ("\n");
#endif /* SIM_DEBUG */

  /* FIXME : TODO : is this necessary? */
  sim_brutal_stop();

  /* FIXME : TODO : is this necessary? */
  speed = fabs(speed);
  accel = fabs(accel);
  deccel = fabs(deccel);

  m_gpio = m_gpio|FLAG_PROPULSION_BUSY_MASK;

  m_me_idx = -1; /* execution disabled */
  m_me_cnt = 0;

  sim_vec_2d_t orig_vec = m_sv.p;
  double orig_theta = m_sv.theta;
  double new_theta = m_sv.theta;
  sim_vec_2d_t target_vec;
  sim_vec_2d_t inv_target_vec;
  bool forward = true;

  target_vec.x = wp[1].x_mm*0.001;
  target_vec.y = wp[1].y_mm*0.001;

  new_theta = sim_compute_theta(orig_vec, target_vec);

  if (fabs(sim_normalize_angle(new_theta-orig_theta))>(M_PI/2))
  {
    forward = false;
  }

  for (int i=1; i<nwp; i++) 
  {
    target_vec.x = wp[i].x_mm*0.001;
    target_vec.y = wp[i].y_mm*0.001;
    inv_target_vec.x = 2.0*orig_vec.x - target_vec.x;
    inv_target_vec.y = 2.0*orig_vec.y - target_vec.y;
    new_theta = sim_compute_theta(orig_vec, target_vec);

    if (forward)
    {
      create_rotation_me(orig_vec, orig_theta, target_vec, 
                         speed, accel, deccel);
      create_translation_me(orig_vec, forward, target_vec, 
                         speed, accel, deccel);
    }
    else
    {
      create_rotation_me(orig_vec, orig_theta, inv_target_vec, 
                         speed, accel, deccel);
      create_translation_me(orig_vec, forward, target_vec, 
                         speed, accel, deccel);
      new_theta = fabs(sim_normalize_angle(M_PI-new_theta));
    }

    orig_theta = new_theta;
    orig_vec = target_vec;
  }

  m_me_idx = 0; /* execution enabled */

  delete wp;
}

void VirtualRobot::on_cmd_execute_point_to(unsigned char *msg_buf, 
                                           size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float speed;
  float accel;
  float deccel;
  strat_way_point_t wp;
  float my_x;
  float my_y;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&(my_x), _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&(my_y), _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  wp.x_mm = my_x*1000.0;
  wp.y_mm = my_y*1000.0;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&speed, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&accel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&deccel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobot::on_cmd_execute_point_to():\n");
  printf ("  speed = %f\n", speed);
  printf ("  accel = %f\n", accel);
  printf ("  deccel = %f\n", deccel);
  printf ("  target:\n");
  printf ("    [%f, %f]\n", wp.x_mm, wp.y_mm);
  printf ("\n");
#endif /* SIM_DEBUG */

  /* FIXME : TODO : is this necessary? */
  sim_brutal_stop();

  /* FIXME : TODO : is this necessary? */
  speed = fabs(speed);
  accel = fabs(accel);
  deccel = fabs(deccel);

  m_gpio = m_gpio|FLAG_PROPULSION_BUSY_MASK;

  m_dbg_duration = 0.0;
#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobot::on_cmd_execute_point_to() : "
          "m_dbg_duration=%f\n", m_dbg_duration);
#endif /* SIM_DEBUG */

  m_me_idx = -1; /* execution disabled */
  m_me_cnt = 0;

  sim_vec_2d_t target_vec;
  target_vec.x = wp.x_mm*0.001;
  target_vec.y = wp.y_mm*0.001;

  create_rotation_me(m_sv.p, m_sv.theta, target_vec, speed, accel, deccel);

  m_me_idx = 0; /* execution enabled */
}

void VirtualRobot::on_cmd_set_pose(unsigned char *msg_buf, 
                                   size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float my_theta;
  float my_x;
  float my_y;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&my_x, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&my_y, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&my_theta, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

#ifdef SIM_DEBUG
  float theta_deg = my_theta*180.0/M_PI;
  float x_mm = my_x*1000.0;
  float y_mm = my_y*1000.0;

  printf ("DEBUG : VirtualRobot::on_cmd_set_pose():\n");
  printf ("  x_mm = %f\n", x_mm);
  printf ("  y_mm = %f\n", y_mm);
  printf ("  theta_deg = %f\n", theta_deg);
  printf ("\n");
#endif /* SIM_DEBUG */

  m_sv.p.x = my_x;
  m_sv.p.y = my_y;
  m_sv.theta = my_theta;
}

void VirtualRobot::on_cmd_start_sequence(unsigned char *msg_buf, 
                                         size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  unsigned short seq_id_s = 0;

  field_len = sizeof(unsigned short int);
  memcpy ((unsigned char *)&seq_id_s, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

#ifdef SIM_DEBUG
  unsigned int seq_id = seq_id_s;
  printf ("DEBUG : VirtualRobot::on_cmd_start_sequence():\n");
  printf ("  seq_id = %u\n", seq_id);
  printf ("\n");
#endif /* SIM_DEBUG */

  /* FIXME : TODO */

}

void VirtualRobot::on_cmd_propulsion_clear_error(unsigned char *msg_buf, 
                                                 size_t msg_len)
{
  msg_buf = msg_buf;
  msg_len = msg_len;

#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobot::on_cmd_propulsion_clear_error()\n");
  printf ("\n");
#endif /* SIM_DEBUG */

  m_gpio &= (~FLAG_PROPULSION_ERROR_MASK);
}

void VirtualRobot::create_me_list_from_strat()
{
  if (!m_default_strat) return;

  if (!m_automatic) 
  {
    printf ("ERROR : calling VirtualRobot::create_me_list_from_strat() "
            "for not automatic robot\n");
    return;
  }

  sim_vec_2d_t iter_p = m_sv.p;
  double iter_theta = m_sv.theta;

  /* FIXME : TODO : is this necessary? */
  sim_brutal_stop();

  m_me_idx = -1; /* execution disabled */
  m_me_cnt = 0;

  /* FIXME : TODO : is this necessary? */
  m_gpio = m_gpio|FLAG_PROPULSION_BUSY_MASK;

  /* FIXME : TODO : is this necessary? */
  m_dbg_duration = 0.0;

  for (int i=0; i<m_default_strat->m_n_actions; i++)
  {
    strat_action_t *act = m_default_strat->m_action_list[i];
    switch (act->h.type) {
    case STRAT_ACTION_TYPE_NONE:
      break;
    case STRAT_ACTION_TYPE_WAIT:
      /* FIXME : TODO */
      printf ("ERROR : action WAIT not yet implemented\n");
      break;

    case STRAT_ACTION_TYPE_TRAJ:
    {
      strat_action_traj_t *act_traj = (strat_action_traj_t *)act;

      /* FIXME : TODO : is this necessary? */
      double speed  = fabs(act_traj->speed);
      double accel  = fabs(act_traj->accel);
      double deccel = fabs(act_traj->deccel);

      sim_vec_2d_t orig_vec = iter_p;
      double orig_theta = iter_theta;
      double new_theta = iter_theta;
      sim_vec_2d_t target_vec;
      sim_vec_2d_t inv_target_vec;
      bool forward = true;

      target_vec.x = act_traj->wp[1].x_mm*0.001;
      target_vec.y = act_traj->wp[1].y_mm*0.001;

      new_theta = sim_compute_theta(orig_vec, target_vec);

      if (fabs(sim_normalize_angle(new_theta-orig_theta))>(M_PI/2))
      {
        forward = false;
      }

      for (int j=1; j<act_traj->nwp; j++) 
      {
        target_vec.x = act_traj->wp[j].x_mm*0.001;
        target_vec.y = act_traj->wp[j].y_mm*0.001;
        inv_target_vec.x = 2.0*orig_vec.x - target_vec.x;
        inv_target_vec.y = 2.0*orig_vec.y - target_vec.y;
        new_theta = sim_compute_theta(orig_vec, target_vec);

        if (forward)
        {
          create_rotation_me(orig_vec, orig_theta, target_vec, 
                             speed, accel, deccel);
          create_translation_me(orig_vec, forward, target_vec, 
                                speed, accel, deccel);
        }
        else
        {
          create_rotation_me(orig_vec, orig_theta, inv_target_vec, 
                             speed, accel, deccel);
          create_translation_me(orig_vec, forward, target_vec, 
                                speed, accel, deccel);
          new_theta = fabs(sim_normalize_angle(M_PI-new_theta));
        }

        orig_theta = new_theta;
        orig_vec = target_vec;
      }

      iter_p = target_vec;
      iter_theta = new_theta;

      break;
    }

    case STRAT_ACTION_TYPE_POINT_TO:
    {
      strat_action_point_to_t *act_point = (strat_action_point_to_t *)act;

      /* FIXME : TODO : is this necessary? */
      double speed  = fabs(act_point->speed);
      double accel  = fabs(act_point->accel);
      double deccel = fabs(act_point->deccel);

      sim_vec_2d_t target_vec;
      target_vec.x = act_point->target.x_mm*0.001;
      target_vec.y = act_point->target.y_mm*0.001;

      create_rotation_me(iter_p, iter_theta, target_vec, 
                         speed, accel, deccel);

      iter_theta = sim_compute_theta(iter_p, target_vec);

      break;
    }

    case STRAT_ACTION_TYPE_NUCLEO_SEQ:
      printf ("ERROR : action NUCLEO_SEQ not simulated\n");
      break;

    case STRAT_ACTION_TYPE_GOTO_ASTAR:
      printf ("ERROR : action GOTO_ASTAR not simulated\n");
      break;

    default:
      printf ("ERROR : bad action type(%d)\n", act->h.type);
    } /* switch (act->h.type) */
  } /* for (int i=0; i<m_default_strat->m_n_actions; i++) */

  m_me_idx = 0; /* execution enabled */
}


VirtualRobotROSImport::VirtualRobotROSImport()
  : VirtualRobot()
{
#ifdef ROS
  m_ros_emul = true;

  goldo::RobotSimulatorConfig simulator_config;
  simulator_config.speed_coeff = 1.7f;
  simulator_config.wheels_spacing = 0.2f;
  simulator_config.encoders_spacing = 0.3f;
  simulator_config.encoders_counts_per_m = 1 / 1.5e-05f;
  m_ros_robot_simulator.setConfig(simulator_config);

  goldo::OdometryConfig odometry_config;
  odometry_config.dist_per_count_left = 1.5e-05f;
  odometry_config.dist_per_count_right = 1.5e-05f;
  odometry_config.wheel_spacing = 0.3f;
  odometry_config.update_period = 1e-3f;
  odometry_config.speed_filter_period = 1e-3f;
  odometry_config.encoder_period = 8192;
  
  m_ros_odometry.setConfig(odometry_config);

  m_ros_propulsion_controller= new goldo::PropulsionController(&m_ros_odometry);

  goldo::PropulsionControllerConfig propulsion_controller_config;
  memset(&propulsion_controller_config,0,sizeof(propulsion_controller_config));

  goldo::PropulsionLowLevelControllerConfig llc;
  goldo::PIDConfig pid1;
  goldo::PIDConfig pid2;

  pid1.period=1e-3f;
  pid1.kp=5;
  pid1.ki=1;
  pid1.kd=0;
  pid1.feed_forward=0.5f;
  pid1.lim_iterm=0.5;
  pid1.lim_dterm=0;
  pid1.min_output=-1.0f;
  pid1.max_output=1.0f;

  pid2.period=1e-3f;
  pid2.kp=0.1;
  pid2.ki=0;
  pid2.kd=0;
  pid2.feed_forward=0.01f;
  pid2.lim_iterm=0.5;
  pid2.lim_dterm=0;
  pid2.min_output=-1.0f;
  pid2.max_output=1.0f;

  llc.speed_pid_config = pid1;
  llc.longi_pid_config = pid1;
  llc.yaw_rate_pid_config = pid2;
  llc.yaw_pid_config = pid2;

  propulsion_controller_config.low_level_config_static = llc;
  propulsion_controller_config.low_level_config_cruise = llc;
  propulsion_controller_config.low_level_config_rotate = llc;
  
  propulsion_controller_config.lookahead_distance = 0.1f;
  propulsion_controller_config.lookahead_time = 0.1f;
  propulsion_controller_config.static_pwm_limit = 1.0f;
  propulsion_controller_config.cruise_pwm_limit = 1.0f;

  m_ros_propulsion_controller->setConfig(propulsion_controller_config);

  auto encoder_values = m_ros_robot_simulator.readEncoders();
  m_ros_odometry.reset(std::get<0>(encoder_values),std::get<1>(encoder_values));
  
  m_ros_update_propulsion_time = 0.0;
#else
  m_ros_emul = true;
#endif
}

int VirtualRobotROSImport::read_yaml_conf(YAML::Node &yconf)
{
  if(VirtualRobot::read_yaml_conf(yconf)!=0) {
    return -1;
  }

#ifdef ROS
  m_ros_propulsion_controller->resetPose(m_sv.p.x, m_sv.p.y, m_sv.theta);

  /* execution enabled */
  m_ros_robot_simulator.setMotorsEnable(true);
  m_ros_propulsion_controller->setEnable(true);
#endif

  return 0;
}

void VirtualRobotROSImport::sim_update(double t_inc)
{
  if (m_ros_emul)
  {
#ifdef ROS
    if ((m_ros_update_propulsion_time+t_inc)>ROS_PROPULSION_STEP_PERIOD)
    {
      m_ros_robot_simulator.doStep();
      auto encoder_values = m_ros_robot_simulator.readEncoders();
      m_ros_odometry.update(std::get<0>(encoder_values), 
                            std::get<1>(encoder_values));
      m_ros_propulsion_controller->update();
      if(m_ros_propulsion_controller->state() != 
         goldo::PropulsionController::State::Inactive)
      {
        m_ros_robot_simulator.setMotorsPwm(
          m_ros_propulsion_controller->leftMotorPwm(), 
          m_ros_propulsion_controller->rightMotorPwm());
        switch(m_ros_propulsion_controller->state())
        {
        case goldo::PropulsionController::State::FollowTrajectory:
        case goldo::PropulsionController::State::Rotate:
        case goldo::PropulsionController::State::Reposition:
          m_gpio = m_gpio|FLAG_PROPULSION_BUSY_MASK;
          break;
        case goldo::PropulsionController::State::ManualControl:
        case goldo::PropulsionController::State::Stopped:
        case goldo::PropulsionController::State::EmergencyStop:
        case goldo::PropulsionController::State::Error:
        default:
          m_gpio = m_gpio&(~FLAG_PROPULSION_BUSY_MASK);
          break;
        }
      }
      else
      {
        m_gpio = m_gpio&(~FLAG_PROPULSION_BUSY_MASK);
      }

      m_ros_update_propulsion_time += t_inc - ROS_PROPULSION_STEP_PERIOD;
    }
    else
    {
      m_ros_update_propulsion_time += t_inc;
    }

	  auto odometry_pose = m_ros_odometry.pose();
	  
    m_sv.v.x     = odometry_pose.speed * cos(m_sv.theta);
    m_sv.v.y     = odometry_pose.speed * sin(m_sv.theta);
    m_sv.v_theta = odometry_pose.yaw_rate;

    //m_sv.p.x     = odometry_pose.position.x;
    //m_sv.p.y     = odometry_pose.position.y;
    //m_sv.theta   = odometry_pose.yaw;
#else
    m_sv.v.x     = 0.0;
    m_sv.v.y     = 0.0;
    m_sv.v_theta = 0.0;
#endif

    m_sv.p.x     += m_sv.v.x*t_inc;
    m_sv.p.y     += m_sv.v.y*t_inc;
    m_sv.theta   += m_sv.v_theta*t_inc;
  }
  else
  {
    VirtualRobot::sim_update(t_inc);
  }
}

void VirtualRobotROSImport::on_cmd_execute_trajectory(
  unsigned char *msg_buf, size_t msg_len)
{
  //VirtualRobot::on_cmd_execute_trajectory(msg_buf, msg_len);

  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float speed;
  float accel;
  float deccel;
  int nwp;
  strat_way_point_t *wp;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&speed, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&accel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&deccel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  nwp = msg_len/(2*sizeof(float));

  wp = new strat_way_point_t[nwp];

  for (int i=0; i<nwp; i++) 
  {
    float my_x;
    float my_y;

    field_len = sizeof(float);
    memcpy ((unsigned char *)&(my_x), _pc, field_len);
    _pc += field_len;
    msg_len -= field_len;

    field_len = sizeof(float);
    memcpy ((unsigned char *)&(my_y), _pc, field_len);
    _pc += field_len;
    msg_len -= field_len;

    wp[i].x_mm = my_x*1000.0;
    wp[i].y_mm = my_y*1000.0;
  }

  if (nwp<2)
  {
    delete wp;
    return;
  }

#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobotROSImport::on_cmd_execute_trajectory():\n");
  printf ("  speed = %f\n", speed);
  printf ("  accel = %f\n", accel);
  printf ("  deccel = %f\n", deccel);
  printf ("  nwp = %d\n", nwp);
  printf ("  wp:\n");
  for (int i=0; i<nwp; i++) 
  {
    printf ("    [%f, %f]\n", wp[i].x*1000.0, wp[i].y*1000.0);
  }
  printf ("\n");
#endif /* SIM_DEBUG */

#ifdef ROS
  std::vector<goldo::Vector2D> points;
  for (int i=0; i<nwp; i++) 
  {
    points.push_back(goldo::Vector2D{wp[i].x_mm, wp[i].y_mm});
    m_ros_propulsion_controller->executeTrajectory(
      points.data(), points.size(), speed, accel, deccel);
  }
#endif /* SIM_DEBUG */

  delete wp;
}

void VirtualRobotROSImport::on_cmd_execute_point_to(
  unsigned char *msg_buf, size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float speed;
  float accel;
  float deccel;
  strat_way_point_t wp;
  float my_x;
  float my_y;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&(my_x), _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&(my_y), _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  wp.x_mm = my_x*1000.0;
  wp.y_mm = my_y*1000.0;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&speed, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&accel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&deccel, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobot::on_cmd_execute_point_to():\n");
  printf ("  speed = %f\n", speed);
  printf ("  accel = %f\n", accel);
  printf ("  deccel = %f\n", deccel);
  printf ("  target:\n");
  printf ("    [%f, %f]\n", wp.x_mm, wp.y_mm);
  printf ("\n");
#endif /* SIM_DEBUG */

  /* FIXME : TODO : is this necessary? */
  sim_brutal_stop();

  /* FIXME : TODO : is this necessary? */
  speed = fabs(speed);
  accel = fabs(accel);
  deccel = fabs(deccel);

  m_gpio = m_gpio|FLAG_PROPULSION_BUSY_MASK;

  m_dbg_duration = 0.0;
#ifdef SIM_DEBUG
  printf ("DEBUG : VirtualRobot::on_cmd_execute_point_to() : "
          "m_dbg_duration=%f\n", m_dbg_duration);
#endif /* SIM_DEBUG */

#ifdef ROS
  float target_vec_x = wp.x_mm*0.001;
  float target_vec_y = wp.y_mm*0.001;

  double yaw_rate   = speed/m_prop_semi_axis;
  double ang_accel  = accel/m_prop_semi_axis;
  double ang_deccel = deccel/m_prop_semi_axis;

  m_ros_propulsion_controller->executePointTo(
    goldo::Vector2D{target_vec_x, target_vec_y}, 
    yaw_rate, ang_accel, ang_deccel);
#else
  wp = wp;
#endif
}

void VirtualRobotROSImport::on_cmd_set_pose(
  unsigned char *msg_buf, size_t msg_len)
{
  VirtualRobot::on_cmd_set_pose(msg_buf, msg_len);

#ifdef ROS
  m_ros_propulsion_controller->resetPose(m_sv.p.x, m_sv.p.y, m_sv.theta);
#endif
}

void VirtualRobotROSImport::on_cmd_propulsion_clear_error(
  unsigned char *msg_buf, size_t msg_len)
{
  VirtualRobot::on_cmd_propulsion_clear_error(msg_buf, msg_len);

#ifdef ROS
  m_ros_propulsion_controller->clearError();
#endif
}


