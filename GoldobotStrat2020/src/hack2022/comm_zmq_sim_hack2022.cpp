#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#ifndef WIN32
#include <termios.h>
#endif
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>

#include "world_state.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "comm_rplidar.hpp"
#include "strat/robot_strat.hpp"

#define ROBOT_SIM 1

#include "sim/virtual_robot.hpp"

#include "message_types_2022.hpp"

using namespace goldobot;


struct MessageHeader {
    uint8_t comm_id{0};
    uint8_t reserved{0};
    uint16_t message_type{0};
    uint32_t time_seconds;
    int32_t time_nanoseconds;
};

enum class CommandEvent : uint8_t { Begin = 0, End, Error, Cancel, Ack };

#define HACK_ACCEL      1.0
#define HACK_YAW_ACCEL 20.0

static int hack_send(uint16_t message_type, const void *buf, size_t len);
static int hack_cmd_traj(unsigned char *_pc, goldo_vec_2d_t *_wp, int _nwp, float speed, float accel, float deccel);
static void simulate_tirette();
static void send_robot_config_load_status();
static void send_servo_ack(uint16_t seq);
static void send_command_event(uint16_t seq, CommandEvent event);
static void translate_propulsion_set_pose(unsigned char *_buf, size_t _len);
static void translate_execute_move_to(unsigned char *_buf, size_t _len);
static void translate_execute_point_to(unsigned char *_buf, size_t _len);
static void translate_execute_point_to_back(unsigned char *_buf, size_t _len);
static void translate_execute_face_direction(unsigned char *_buf, size_t _len);
static void translate_execute_trajectory(unsigned char *_buf, size_t _len);

CommZmq CommZmq::s_instance;

CommZmq& CommZmq::instance()
{
	return s_instance;
}

CommZmq::CommZmq()
{
  strncpy(m_thread_name,"CommZmq",sizeof(m_thread_name));

  m_stop_task = false;
  m_task_running = false;

  m_zmq_context = NULL;

  m_pub_socket = NULL;
  m_pull_socket = NULL;

  m_pub1_socket = NULL;
  m_pull1_socket = NULL;
}

int CommZmq::init(int port_nb)
{
  int rc;
  char char_buff[64];

  port_nb = port_nb;

  m_zmq_context = zmq_init(1);

  m_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
  if (m_pub_socket<0) {
    printf ("pub_socket : cannot create ZMQ_PUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", 3001);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_bind(m_pub_socket, char_buff);
  if (rc<0) {
    printf ("pub_socket : cannot bind ZMQ_PUB socket\n");
    return -1;
  }

  m_pull_socket = zmq_socket(m_zmq_context, ZMQ_SUB);
  if (m_pull_socket<0) {
    printf ("pull_socket : cannot create ZMQ_SUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", 3002);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_bind(m_pull_socket, char_buff);
  if (rc<0) {
    printf ("pull_socket : cannot bind ZMQ_SUB socket\n");
    return -1;
  }
  zmq_setsockopt(m_pull_socket,ZMQ_SUBSCRIBE, "", 0);

  m_pub1_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
  if (m_pub1_socket<0) {
    printf ("pub1_socket : cannot create ZMQ_PUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", 3102);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_bind(m_pub1_socket, char_buff);
  if (rc<0) {
    printf ("pub_socket : cannot bind ZMQ_PUB socket\n");
    return -1;
  }

  return 0;
}

#define N_PULL_SOCKETS 1

void CommZmq::taskFunction()
{
  struct timespec curr_tp;
  int curr_time_ms = 0;
  int old_time_ms = 0;
  uint16_t active_propulsion_seq = 0;

  zmq_pollitem_t poll_items[N_PULL_SOCKETS];

  poll_items[0].socket = m_pull_socket;
  poll_items[0].fd = 0;
  poll_items[0].events = ZMQ_POLLIN;

  m_task_running = true;

  while(!m_stop_task)
  {
    zmq_poll (poll_items, N_PULL_SOCKETS, 100);

    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if (curr_time_ms > (old_time_ms + 100)) {
      /* FIXME : TODO : send some heartbeat message? */

      old_time_ms = curr_time_ms;
    }

    if(poll_items[0].revents && ZMQ_POLLIN) /* goldobot_ihm (control) messages */
    {            
      unsigned char buff[1024];
      size_t bytes_read = 0;
      int64_t more=1;
      size_t more_size = sizeof(more);
      while(more)
      {
        bytes_read += zmq_recv(m_pull_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);           
        zmq_getsockopt(m_pull_socket, ZMQ_RCVMORE, &more, &more_size);
      }
      buff[bytes_read] = 0;
      MessageHeader message_header;
      memcpy (&message_header, buff, sizeof(MessageHeader));
      unsigned char *message_body = buff + sizeof(MessageHeader);
      size_t message_body_size = bytes_read - sizeof(MessageHeader);
      uint16_t seq;

#if 0 /* FIXME : DEBUG */
      printf("  ZMQ DEBUG: received message:\n");
      printf("  ");
      {
        int i;
        for (i=0; i<(int)bytes_read; i++) printf ("%.2x ",buff[i]);
        printf ("\n");
      }
#endif

      switch (message_header.message_type) {
      case 0:    /* CommUartStats */
      case 1:    /* CommUartPing */
        //printf ("  ZMQ DEBUG: NOT IMPLEMENTED\n");
        break;
      case 11:    /* MatchTimerStart */
        printf ("  ZMQ DEBUG: MatchTimerStart NOT IMPLEMENTED\n");
        break;
      case 41: /* ServoMoveMultiple                */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: ServoMoveMultiple FAKE seq=%d\n", seq);
        send_servo_ack(seq);
        break;
      case 42: /* ServoSetEnable                   */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: ServoSetEnable FAKE seq=%d\n", seq);
        send_servo_ack(seq);
        break;
      case 50: /* ODriveRequestPacket              */
        printf ("  ZMQ DEBUG: ODriveRequestPacket NOT IMPLEMENTED\n");
        break;
      case 100: /* PropulsionEnableSet             */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionEnableSet FAKE seq=%d\n", seq);
        send_command_event(seq, CommandEvent::Ack);
        break;
      case 101: /* PropulsionMotorsEnableSet       */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionMotorsEnableSet FAKE seq=%d\n", seq);
        send_command_event(seq, CommandEvent::Ack);
        break;
      case 104: /* PropulsionSetAccelerationLimits */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionSetAccelerationLimits FAKE seq=%d\n", seq);
        send_command_event(seq, CommandEvent::Ack);
        break;
      case 105: /* PropulsionSetPose               */
        seq = *((uint16_t *)message_body);
        translate_propulsion_set_pose(message_body+sizeof(uint16_t), message_body_size-sizeof(uint16_t));
        printf ("  ZMQ DEBUG: PropulsionSetPose seq=%d\n", seq);
        send_command_event(seq, CommandEvent::Ack);
        break;
      case 108: /* PropulsionClearError            */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionClearError TODO seq=%d\n", seq);
        send_command_event(seq, CommandEvent::Ack);
        break;
      case 110: /* PropulsionSetSimulationMode     */
        printf ("  ZMQ DEBUG: PropulsionSetSimulationMode NOT IMPLEMENTED\n");
        break;
      case 141: /* PropulsionExecuteMoveTo         */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionExecuteMoveTo seq=%d\n", seq);
        translate_execute_move_to(message_body+sizeof(uint16_t), message_body_size-sizeof(uint16_t));
        send_command_event(seq, CommandEvent::Ack);
        active_propulsion_seq = seq;
        break;
      case 143: /* PropulsionExecutePointTo        */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionExecutePointTo seq=%d\n", seq);
        translate_execute_point_to(message_body+sizeof(uint16_t), message_body_size-sizeof(uint16_t));
        send_command_event(seq, CommandEvent::Ack);
        active_propulsion_seq = seq;
        break;
      case 144: /* PropulsionExecuteFaceDirection  */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionExecuteFaceDirection seq=%d\n", seq);
        translate_execute_face_direction(message_body+sizeof(uint16_t), message_body_size-sizeof(uint16_t));
        send_command_event(seq, CommandEvent::Ack);
        active_propulsion_seq = seq;
        break;
      case 145: /* PropulsionExecuteTrajectory    */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionExecuteTrajectory seq=%d\n", seq);
        translate_execute_trajectory(message_body+4, message_body_size-4);
        send_command_event(seq, CommandEvent::Ack);
        active_propulsion_seq = seq;
        break;
      case 153: /* PropulsionExecutePointToBack    */
        seq = *((uint16_t *)message_body);
        printf ("  ZMQ DEBUG: PropulsionExecutePointToBack seq=%d\n", seq);
        translate_execute_point_to_back(message_body+sizeof(uint16_t), message_body_size-sizeof(uint16_t));
        send_command_event(seq, CommandEvent::Ack);
        active_propulsion_seq = seq;
        break;
      case 152: /* PropulsionODriveClearErrors     */
        printf ("  ZMQ DEBUG: PropulsionODriveClearErrors NOT IMPLEMENTED\n");
        break;
      case 200: /* RobotConfigLoadBegin            */
        printf ("  ZMQ DEBUG: RobotConfigLoadBegin NOT IMPLEMENTED\n");
        break;
      case 201: /* RobotConfigLoadChunk            */
        printf ("  ZMQ DEBUG: RobotConfigLoadChunk NOT IMPLEMENTED\n");
        break;
      case 202: /* RobotConfigLoadEnd              */
        printf ("  ZMQ DEBUG: RobotConfigLoadEnd FAKE\n");
        send_robot_config_load_status();
        break;

      case 1024: /* RplidarStart                   */
        printf ("  ZMQ DEBUG: RplidarStart NOT IMPLEMENTED\n");
        //CommRplidar::instance().start_scan();
        break;
      case 1025: /* RplidarStop                    */
        printf ("  ZMQ DEBUG: RplidarStop NOT IMPLEMENTED\n");
        //CommRplidar::instance().stop_scan();
        break;
      case 2048: /* RobotStratDbgStartMatch        */
        printf ("  ZMQ DEBUG: RobotStratDbgStartMatch\n");
        //RobotStrat::instance().start_match(); /* FIXME : TODO */
        simulate_tirette();
        WorldState::instance().start_signal();
        break;
      case 2049: /* RobotStratDbgPauseMatch        */
        printf ("  ZMQ DEBUG: RobotStratDbgPauseMatch NOT IMPLEMENTED\n");
        //RobotStrat::instance().dbg_pause_match();
        break;
      case 2050: /* RobotStratDbgResumeMatch       */
        printf ("  ZMQ DEBUG: RobotStratDbgResumeMatch NOT IMPLEMENTED\n");
        //RobotStrat::instance().dbg_resume_match();
        break;

      default:
        printf ("  ZMQ DEBUG: received command code : %x ; NOT IMPLEMENTED\n", message_header.message_type);
      }
    }

    if (active_propulsion_seq!=0)
    {
      uint32_t my_gpio = VirtualRobots::myself().gpio();
      if (!(my_gpio&VirtualRobot::FLAG_PROPULSION_BUSY_MASK))
      {
        printf ("  ZMQ DEBUG: propulsion command completed seq=%d\n", active_propulsion_seq);
        send_command_event(active_propulsion_seq, CommandEvent::End);
        active_propulsion_seq = 0;
      }
    }

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  }

  zmq_term(m_zmq_context);

  m_task_running = false;
}

int CommZmq::send(const void *buf, size_t len, int flags)
{
  if (m_pub_socket==NULL) return -1;

  char *tmp_buffer = (char *)buf;
  uint16_t message_type = 0;
  MessageHeader header;
  struct timespec tv;

  clock_gettime(CLOCK_MONOTONIC, &tv);

  header.comm_id = 0;
  header.time_seconds = tv.tv_sec;
  header.time_nanoseconds = tv.tv_nsec;

  memcpy (&message_type, buf, sizeof(message_type));

  switch (message_type) {
  case 0x0001: /* Heartbeat */
    header.message_type = 2;
    break;
  case 0x0014: /* SensorsChange */
    header.message_type = 33;
    break;
  case 0x0008: /* PropulsionTelemetry */
    header.message_type = 120;
    break;
  case 0x0009: /* PropulsionTelemetryEx */
    header.message_type = 121;
    break;
  }

  zmq_send(m_pub_socket, (const char*)(&header), sizeof(header), ZMQ_SNDMORE);
  return zmq_send(m_pub_socket, (const char*)(tmp_buffer+2), len-2, 0);
}

static int hack_send(uint16_t message_type, const void *buf, size_t len)
{
  void *m_pub_socket=CommZmq::instance().get_pub_socket();

  if (m_pub_socket==NULL) return -1;

  MessageHeader header;
  struct timespec tv;

  clock_gettime(CLOCK_MONOTONIC, &tv);

  header.comm_id = 0;
  header.time_seconds = tv.tv_sec;
  header.time_nanoseconds = tv.tv_nsec;
  header.message_type = message_type;

  zmq_send(m_pub_socket, (const char*)(&header), sizeof(header), ZMQ_SNDMORE);
  return zmq_send(m_pub_socket, (const char*)(buf), len, 0);
}

int CommZmq::recv(void *buf, size_t len, int flags)
{
  if (m_pull_socket==NULL) return -1;
  return zmq_recv (m_pull_socket, buf, len, flags);
}

void CommZmq::send_robot_detection(const char *buf, size_t len)
{
  if (m_pub1_socket==NULL) return;

  uint8_t message_type = 2;

  zmq_send(m_pub1_socket, (const char*)(&message_type), sizeof(message_type), ZMQ_SNDMORE);
  zmq_send(m_pub1_socket, buf, len, 0);
}

static void simulate_tirette()
{
  unsigned int sensor_state;
  uint16_t message_type = (uint16_t) goldobot::CommMessageType::SensorsState;

  sensor_state = 0;
  hack_send(message_type, &sensor_state, sizeof(sensor_state));

  usleep(1000000);

  sensor_state = 1;
  hack_send(message_type, &sensor_state, sizeof(sensor_state));
}

static void send_robot_config_load_status()
{
  uint8_t load_status;
  uint16_t message_type = (uint16_t) goldobot::CommMessageType::RobotConfigLoadStatus;

  load_status = 0; /* FIXME : DEBUG : WTF?!!.. */
  hack_send(message_type, &load_status, sizeof(load_status));
}

static void send_servo_ack(uint16_t seq)
{
  uint16_t message_type = (uint16_t) goldobot::CommMessageType::ServoAck;

  hack_send(message_type, &seq, sizeof(seq));
}

static void send_command_event(uint16_t seq, CommandEvent event)
{
  uint8_t buff[8]; // timestamp, sequence_number, status, error
  uint16_t message_type = (uint16_t) goldobot::CommMessageType::PropulsionCommandEvent;

  *(uint32_t*)(buff) = 0; // m_current_timestamp
  *(uint16_t*)(buff + 4) = seq;
  buff[6] = static_cast<uint8_t>(event);
  buff[7] = static_cast<uint8_t>(0); // m_controller.error()

  hack_send(message_type, &buff, sizeof(buff));
}

static void translate_propulsion_set_pose(unsigned char *_buf, size_t _len)
{
  float *dbg_payload = (float *)(_buf);
#if 0
  goldo_vec_2d_t my_p = VirtualRobots::myself().sv().p;
  float my_theta = VirtualRobots::myself().sv().theta;
#else
  goldo_vec_2d_t my_p;
  float my_theta;
  my_p.x = dbg_payload[0];
  my_p.y = dbg_payload[1];
  my_theta = dbg_payload[2];
#endif
  printf ("    p=<%5.2f,%5.2f> theta=%f\n", my_p.x*1000.0, my_p.y*1000.0, my_theta*180.0/M_PI);

  WorldState::instance().lock_update(true);
  usleep(10000);
  /* no translation needed only skipping the sequence in the buffer */
  VirtualRobots::myself().on_cmd_set_pose(_buf, 3*sizeof(float));
  usleep(10000);
  WorldState::instance().lock_update(false);
  usleep(10000);
}

static int hack_cmd_traj(unsigned char *_pc, goldo_vec_2d_t *_wp, int _nwp, float speed, float accel, float deccel)
{
  int cmd_buf_len = 0;
  int field_len = 0;

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
    float my_x = _wp[i].x;
    float my_y = _wp[i].y;

    field_len = sizeof(float);
    memcpy (_pc, (unsigned char *)&(my_x), field_len);
    _pc += field_len;
    cmd_buf_len += field_len;

    field_len = sizeof(float);
    memcpy (_pc, (unsigned char *)&(my_y), field_len);
    _pc += field_len;
    cmd_buf_len += field_len;
  }

  return cmd_buf_len;
}

static void translate_execute_move_to(unsigned char *_buf, size_t _len)
{
  float *dbg_payload = (float *)(_buf);
  goldo_vec_2d_t my_wp[2];
  my_wp[0] = VirtualRobots::myself().sv().p;
  my_wp[1].x = dbg_payload[0];
  my_wp[1].y = dbg_payload[1];
  float speed;
  speed = dbg_payload[2];
  printf ("    speed=%f\n", speed);
  printf ("    origin_pos=<%5.2f,%5.2f>\n", my_wp[0].x*1000.0, my_wp[0].y*1000.0);
  printf ("    target_pos=<%5.2f,%5.2f>\n", my_wp[1].x*1000.0, my_wp[1].y*1000.0);
  unsigned char new_buf[64];
  int cmd_buf_len = hack_cmd_traj(new_buf, my_wp, 2, speed, HACK_ACCEL, HACK_ACCEL);
  VirtualRobots::myself().on_cmd_execute_trajectory(new_buf, cmd_buf_len);
}

int hack_cmd_point_to(unsigned char *_pc, goldo_vec_2d_t *_wp, float speed, float accel, float deccel)
{
  int cmd_buf_len = 0;
  int field_len = 0;

  float my_x = _wp->x;
  float my_y = _wp->y;

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

  return cmd_buf_len;
}

static void translate_execute_point_to(unsigned char *_buf, size_t _len)
{
  float *dbg_payload = (float *)(_buf);
  goldo_vec_2d_t my_target;
  my_target.x = dbg_payload[0];
  my_target.y = dbg_payload[1];
  float yaw_rate;
  yaw_rate = dbg_payload[2];
  printf ("    yaw_rate=%f\n", yaw_rate);
  printf ("    target=<%5.2f,%5.2f>\n", my_target.x*1000.0, my_target.y*1000.0);
  unsigned char new_buf[64];
  int cmd_buf_len = hack_cmd_point_to(new_buf, &my_target, yaw_rate, HACK_YAW_ACCEL, HACK_YAW_ACCEL);
  VirtualRobots::myself().on_cmd_execute_point_to(new_buf, cmd_buf_len);
}

static void translate_execute_point_to_back(unsigned char *_buf, size_t _len)
{
  float *dbg_payload = (float *)(_buf);
  goldo_vec_2d_t my_target;
  my_target.x = dbg_payload[0];
  my_target.y = dbg_payload[1];
  goldo_vec_2d_t my_p = VirtualRobots::myself().sv().p;
  float diff_x = (my_target.x - my_p.x);
  float diff_y = (my_target.y - my_p.y);
  my_target.x = my_p.x - diff_x;
  my_target.y = my_p.y - diff_y;
  float yaw_rate = dbg_payload[2];
  printf ("    yaw_rate=%f\n", yaw_rate);
  printf ("    target=<%5.2f,%5.2f>\n", my_target.x*1000.0, my_target.y*1000.0);
  unsigned char new_buf[64];
  int cmd_buf_len = hack_cmd_point_to(new_buf, &my_target, yaw_rate, HACK_YAW_ACCEL, HACK_YAW_ACCEL);
  VirtualRobots::myself().on_cmd_execute_point_to(new_buf, cmd_buf_len);
}

static void translate_execute_face_direction(unsigned char *_buf, size_t _len)
{
  float *dbg_payload = (float *)(_buf);
  float yaw_target = dbg_payload[0];
  float yaw_rate = dbg_payload[1];
  goldo_vec_2d_t my_p = VirtualRobots::myself().sv().p;
  goldo_vec_2d_t my_target;
  my_target.x = my_p.x + 0.1*cos(yaw_target);
  my_target.y = my_p.y + 0.1*sin(yaw_target);
  printf ("    yaw_target=%f\n", yaw_target*180/M_PI);
  printf ("    yaw_rate=%f\n", yaw_rate);
  printf ("    target=<%5.2f,%5.2f>\n", my_target.x*1000.0, my_target.y*1000.0);
  unsigned char new_buf[64];
  int cmd_buf_len = hack_cmd_point_to(new_buf, &my_target, yaw_rate, HACK_YAW_ACCEL, HACK_YAW_ACCEL);
  VirtualRobots::myself().on_cmd_execute_point_to(new_buf, cmd_buf_len);
}

static void translate_execute_trajectory(unsigned char *_buf, size_t _len)
{
  goldo_vec_2d_t my_wp[128];
  float *dbg_payload = (float *)(_buf);
  float speed               = *(dbg_payload++);
  float reposition_distance = *(dbg_payload++);
  float reposition_speed    = *(dbg_payload++);
  int np = (_len - (sizeof(float)*3))/(2*sizeof(float)) + 1;
  my_wp[0] = VirtualRobots::myself().sv().p;
  for (int i=1; i<np; i++)
  {
    my_wp[i].x = *(dbg_payload++);
    my_wp[i].y = *(dbg_payload++);
  }
  printf ("    speed=%f\n", speed);
  printf ("    reposition_distance=%f\n", reposition_distance);
  printf ("    reposition_speed=%f\n", reposition_speed);
  printf ("    np=%d\n", np);
  for (int i=0; i<np; i++)
  {
    printf ("    wp[%d]=<%5.2f,%5.2f>\n", i, my_wp[i].x*1000.0, my_wp[i].y*1000.0);
  }
  unsigned char new_buf[512];
  int cmd_buf_len = hack_cmd_traj(new_buf, my_wp, np, speed, HACK_ACCEL, HACK_ACCEL);
  VirtualRobots::myself().on_cmd_execute_trajectory(new_buf, cmd_buf_len);
}

