#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#define ROBOT_SIM 1

#include "robot_state.hpp"
#include "sim/virtual_robot.hpp"

using namespace goldobot;

VirtualRobot::VirtualRobot()
{
  memset(&m_recv_buf, 0, sizeof(m_recv_buf));
  /* FIXME : TODO */
}

void VirtualRobot::sim_receive(const unsigned char *msg_buf, size_t msg_len)
{
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

void VirtualRobot::on_cmd_execute_trajectory(unsigned char *msg_buf, 
                                             size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float speed;
  float accel;
  float deccel;
  int _nwp;
  strat_way_point_t *_wp;

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

  _nwp = field_len/(2*sizeof(float));

  _wp = new strat_way_point_t[_nwp];

  for (int i=0; i<_nwp; i++) 
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

    _wp[i].x_mm = my_x*1000.0;
    _wp[i].y_mm = my_y*1000.0;
  }

  /* FIXME : TODO */

  delete _wp;
}

void VirtualRobot::on_cmd_execute_point_to(unsigned char *msg_buf, 
                                           size_t msg_len)
{
  unsigned char *_pc = msg_buf;
  int field_len = 0;
  float speed;
  float accel;
  float deccel;
  strat_way_point_t _wp;
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

  _wp.x_mm = my_x*1000.0;
  _wp.y_mm = my_y*1000.0;

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


  /* FIXME : TODO */
  _wp = _wp;

}

void VirtualRobot::on_cmd_set_pose(unsigned char *msg_buf, 
                                   size_t msg_len)
{
  /* FIXME : TODO */
  msg_buf = msg_buf;
  msg_len = msg_len;
}

void VirtualRobot::on_cmd_start_sequence(unsigned char *msg_buf, 
                                         size_t msg_len)
{
  /* FIXME : TODO */
  msg_buf = msg_buf;
  msg_len = msg_len;
}

void VirtualRobot::on_cmd_propulsion_clear_error(unsigned char *msg_buf, 
                                                 size_t msg_len)
{
  /* FIXME : TODO */
  msg_buf = msg_buf;
  msg_len = msg_len;
}
