#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>


#define ROBOT_SIM 1
#define SIM_DEBUG 1

#include "robot_state.hpp"
#include "sim/virtual_robot.hpp"

using namespace goldobot;

VirtualRobot VirtualRobot::s_myself;

VirtualRobot& VirtualRobot::myself()
{
  return s_myself;
}

VirtualRobot VirtualRobot::s_partner;

VirtualRobot& VirtualRobot::partner()
{
  return s_partner;
}

VirtualRobot VirtualRobot::s_adversary1;

VirtualRobot& VirtualRobot::adversary1()
{
  return s_adversary1;
}

VirtualRobot VirtualRobot::s_adversary2;

VirtualRobot& VirtualRobot::adversary2()
{
  return s_adversary2;
}


VirtualRobot::VirtualRobot()
{
  m_enabled = false;

  m_automatic = true;

  memset(&m_sv, 0, sizeof(m_sv));

  m_gpio = GPIO_START_MASK;

  default_strat = new StratTask();

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
}

int VirtualRobot::read_yaml_conf(YAML::Node &yconf)
{
  /* FIXME : TODO */
  return -1;
}

void VirtualRobot::sim_update(double t_inc)
{
  m_sv.p.x   += m_sv.v.x*t_inc;
  m_sv.p.y   += m_sv.v.y*t_inc;
  m_sv.theta += m_sv.v_theta*t_inc;

  if ((m_me!=NULL) && (m_me_idx!=-1) && (m_me_idx!=m_me_cnt))
  {
    if (m_me[m_me_cnt].type == SM_ELEM_DYN)
    {
      double a_x     = m_me[m_me_idx].u.dyn.lin_accel * cos(m_sv.theta);
      double a_y     = m_me[m_me_idx].u.dyn.lin_accel * sin(m_sv.theta);
      double a_theta = m_me[m_me_idx].u.dyn.ang_accel;

      m_sv.v.x     += a_x*t_inc;
      m_sv.v.y     += a_y*t_inc;
      m_sv.v_theta += a_theta*t_inc;

      m_me[m_me_idx].u.dyn.duration -= t_inc;
      if (m_me[m_me_idx].u.dyn.duration<0.0) m_me_idx++;
    }
    else
    {
      /* FIXME : TODO */
      m_me_idx++;
    }
  }
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
  double new_theta = sim_compute_theta(orig, target);
  double d_theta = sim_normalize_angle(new_theta - orig_theta);

  speed  = speed/m_prop_semi_axis;
  accel  = accel/m_prop_semi_axis;
  deccel = deccel/m_prop_semi_axis;

  double t_a;
  double t_i;
  double t_d;
  sim_compute_motion_times(fabs(d_theta), speed, accel, deccel, t_a, t_i, t_d);

  if (d_theta<0)
  {
    speed  = -speed;
    accel  = -accel;
    deccel = -deccel;
  }

  m_me[m_me_cnt].type = SM_ELEM_DYN;
  m_me[m_me_cnt].u.dyn.duration  = t_a;
  m_me[m_me_cnt].u.dyn.lin_accel = 0.0;
  m_me[m_me_cnt].u.dyn.ang_accel = accel/m_prop_semi_axis;
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
  m_me[m_me_cnt].u.dyn.ang_accel = deccel/m_prop_semi_axis;
  m_me_cnt++;
}

void VirtualRobot::create_translation_me(sim_vec_2d_t &orig, 
                                         sim_vec_2d_t &target,
                                         float speed, float accel, float deccel)
{
  double dist = sim_dist(target, orig);

  double t_a;
  double t_i;
  double t_d;
  sim_compute_motion_times(dist, speed, accel, deccel, t_a, t_i, t_d);

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
  m_me[m_me_cnt].u.dyn.lin_accel = 0.0;
  m_me[m_me_cnt].u.dyn.ang_accel = deccel;
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


  /* FIXME : TODO */

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
  sim_vec_2d_t target_vec;

  for (int i=0; i<nwp; i++) 
  {
    target_vec.x = wp[i].x_mm*0.001;
    target_vec.y = wp[i].y_mm*0.001;

    create_rotation_me(orig_vec, orig_theta, target_vec, speed, accel, deccel);

    orig_theta = sim_compute_theta(orig_vec, target_vec);
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
  float x_mm;
  float y_mm;
  float theta_deg;
  float my_x;
  float my_y;
  float my_theta;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&my_x, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&my_y, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  x_mm = my_x*1000.0;
  y_mm = my_y*1000.0;

  field_len = sizeof(float);
  memcpy ((unsigned char *)&my_theta, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  theta_deg = my_theta*180.0/M_PI;

#ifdef SIM_DEBUG
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
  unsigned int seq_id = 0;

  field_len = sizeof(unsigned short int);
  memcpy ((unsigned char *)&seq_id_s, _pc, field_len);
  _pc += field_len;
  msg_len -= field_len;

  seq_id = seq_id_s;

#ifdef SIM_DEBUG
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
