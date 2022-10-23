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

#include "goldo_conf.hpp"
#include "world_state.hpp"
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "comm_rplidar.hpp"
#include "strat/robot_strat.hpp"


using namespace goldobot;


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

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  m_pub1_socket = NULL;
  m_pull1_socket = NULL;
#endif
}

int CommZmq::init(int port_nb)
{
  int rc;
  char char_buff[64];

  m_zmq_context = zmq_init(1);

  m_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
  if (m_pub_socket<0) {
    printf ("pub_socket : cannot create ZMQ_PUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", port_nb);
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

  sprintf(char_buff, "tcp://*:%d", port_nb+1);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_bind(m_pull_socket, char_buff);
  if (rc<0) {
    printf ("pull_socket : cannot bind ZMQ_SUB socket\n");
    return -1;
  }
  zmq_setsockopt(m_pull_socket,ZMQ_SUBSCRIBE, "", 0);

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  m_pull1_socket = zmq_socket(m_zmq_context, ZMQ_SUB);
  if (m_pull1_socket<0) {
    printf ("detect_socket : cannot create ZMQ_SUB socket\n");
    return -1;
  }

  rc = zmq_connect(m_pull1_socket, "tcp://127.0.0.1:3202");
  if (rc<0) {
    printf ("zmq_connect() error\n");
  }
  zmq_setsockopt(m_pull1_socket,ZMQ_SUBSCRIBE, "", 0);
#endif

  return 0;
}

void CommZmq::taskFunction()
{
  goldo_conf_info_t& ci = GoldoConf::instance().c();

  struct timespec curr_tp;
  int curr_time_ms = 0;

  int dbg_cnt = 0;
  //unsigned int dbg_color_code = 0;
  unsigned int dbg_id_code = 0;
  double dbg_x_rel = 0.0;
  double dbg_y_rel = 0.0;
  double dbg_x_abs = 0.0;
  double dbg_y_abs = 0.0;


  zmq_pollitem_t poll_items[2]; /* FIXME : DEBUG : HACK CRIDF2021 (was 1) */

  poll_items[0].socket = m_pull_socket;
  poll_items[0].fd = 0;
  poll_items[0].events = ZMQ_POLLIN;
#if 1 /* FIXME : DEBUG : HACK CRIDF2021 */
  poll_items[1].socket = m_pull1_socket;
  poll_items[1].fd = 0;
  poll_items[1].events = ZMQ_POLLIN;
#endif

  m_task_running = true;

  while(!m_stop_task)
  {
    zmq_poll (poll_items, 2, 100); /* FIXME : DEBUG : HACK CRIDF2021 (was 1) */

    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if(poll_items[0].revents && ZMQ_POLLIN)
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
      uint16_t message_type = 0;
      memcpy (&message_type, buff, sizeof(message_type));

#if 0 /* FIXME : DEBUG */
      printf("  ZMQ DEBUG: received message_type = %d\n", message_type);
      printf("  ");
      {
        int i;
        for (i=0; i<(int)bytes_read; i++) printf ("%.2x ",buff[i]);
        printf ("\n");
      }
#endif

      /* FIXME : TODO : import message_types.h into the project */
      switch (message_type) {
      case 55:   /* DbgPropulsionExecuteTrajectory */
        printf ("  ZMQ DEBUG: DbgPropulsionExecuteTrajectory\n");
        DirectUartNucleo::instance().send(buff, bytes_read);
        break;
      case 99:   /* PropulsionClearError */
        printf ("  ZMQ DEBUG: PropulsionClearError\n");
        DirectUartNucleo::instance().send(buff, bytes_read);
        break;
      case 1024: /* RplidarStart                   */
        printf ("  ZMQ DEBUG: RplidarStart\n");
        CommRplidar::instance().start_scan();
        break;
      case 1025: /* RplidarStop                    */
        printf ("  ZMQ DEBUG: RplidarStop\n");
        CommRplidar::instance().stop_scan();
        break;
      case 2048: /* RobotStratDbgStartMatch        */
        printf ("  ZMQ DEBUG: RobotStratDbgStartMatch\n");
        RobotStrat::instance().start_match();
        WorldState::instance().start_signal();
        break;
      case 2049: /* RobotStratDbgPauseMatch        */
        printf ("  ZMQ DEBUG: RobotStratDbgPauseMatch\n");
        RobotStrat::instance().dbg_pause_match();
        break;
      case 2050: /* RobotStratDbgResumeMatch       */
        printf ("  ZMQ DEBUG: RobotStratDbgResumeMatch\n");
        RobotStrat::instance().dbg_resume_match();
        break;
      }
    } /* if(poll_items[0].revents && ZMQ_POLLIN) */

#if 1 /* FIXME : DEBUG : HACK CRIDF2021 & DEMO2022 */
    WorldState::instance().lock();
    //int obj_id = WorldState::instance().detected_object(0).id;
    int obj_time_ms = WorldState::instance().detected_object(0).timestamp_ms;
    if((WorldState::instance().s().n_detected_objects > 0) && (curr_time_ms > (obj_time_ms+1000)))
    {
      WorldState::instance().s().n_detected_objects = 0;
      WorldState::instance().detected_object(0).id = 0;
      //printf ("CLEAN\n");
    }
    WorldState::instance().release();

    if(poll_items[1].revents && ZMQ_POLLIN)
    {            
      unsigned char buff[1024];
      size_t bytes_read = 0;
      int64_t more=1;
      size_t more_size = sizeof(more);
      while(more)
      {
        bytes_read += zmq_recv(m_pull1_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);
        zmq_getsockopt(m_pull1_socket, ZMQ_RCVMORE, &more, &more_size);
      }
      if (bytes_read==16)
      {
        int *buff_i = (int *)buff;
        if (buff_i[0]==0x7d7f1892)
        {
          RobotState::instance().lock();
          int l_odo_x_mm      = RobotState::instance().s().x_mm;
          int l_odo_y_mm      = RobotState::instance().s().y_mm;
          int l_odo_theta_deg = RobotState::instance().s().theta_deg;
          RobotState::instance().release();
          double l_odo_theta_rad = l_odo_theta_deg*M_PI/180.0;

          double cos_theta = cos(l_odo_theta_rad);
          double sin_theta = sin(l_odo_theta_rad);

          //unsigned int color_code = buff_i[1];
          unsigned int id_code = buff_i[1];
          double x_rel = buff_i[2];
          double y_rel = buff_i[3];

          //dbg_color_code = color_code;
          dbg_id_code = id_code;
          dbg_x_rel = x_rel;
          dbg_y_rel = y_rel;

          double x_abs = x_rel*cos_theta - y_rel*sin_theta + l_odo_x_mm;
          double y_abs = x_rel*sin_theta + y_rel*cos_theta + l_odo_y_mm;

          struct timespec curr_tp;
          clock_gettime(1, &curr_tp);
          int detect_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

#if 0 /* HACK CRIDF2021 */
          if ((x_abs>150.0) && (x_abs<1600.0) && (y_abs>-800.0) && (y_abs<800.0) && ((color_code==1) || (color_code==2)))
          {
            WorldState::instance().lock();
            WorldState::instance().detected_object(0).timestamp_ms = detect_time_ms;
            WorldState::instance().detected_object(0).id = 1;
            WorldState::instance().detected_object(0).attr = color_code;
            WorldState::instance().detected_object(0).x_mm = x_abs;
            WorldState::instance().detected_object(0).y_mm = y_abs;
            WorldState::instance().release();
          }
#endif

#if 1 /* HACK DEMO2022 */
          if ((x_abs>1200.0) && (x_abs<1550.0) && (y_abs>-700.0) && (y_abs<700.0) &&
              ((id_code==47) || (id_code==13) || (id_code==36) || (id_code==17)))
          {
            WorldState::instance().lock();
            WorldState::instance().detected_object(0).timestamp_ms = detect_time_ms;
            WorldState::instance().detected_object(0).id = id_code;
            WorldState::instance().detected_object(0).attr = 0;
            WorldState::instance().detected_object(0).x_mm = x_abs;
            WorldState::instance().detected_object(0).y_mm = y_abs;
            WorldState::instance().release();
          }

          WorldState::instance().s().observable[0].value = false;
          switch (id_code)
          {
          case 47: /* RED */
            break;
          case 13: /* BLUE */
            WorldState::instance().s().observable[0].value = false;
            WorldState::instance().s().observable[0].x_mm  = x_abs;
            WorldState::instance().s().observable[0].y_mm  = y_abs;
            break;
          case 36: /* GREEN */
            break;
          case 17: /* NONE */
            break;
          }
#endif
        }
        else
        {
          printf ("DETECT ERROR (coockie=%x)\n", buff_i[0]);
        }
      }
      else
      {
        printf ("DETECT ERROR (bytes_read=%d)\n", (int)bytes_read);
      }
    } /* if(poll_items[1].revents && ZMQ_POLLIN) */
#endif /* FIXME : DEBUG : HACK CRIDF2021 & DEMO2022 */

#if 1 /* FIXME : DEBUG : DISPLAY */
    if(dbg_cnt==1)
    {
      if((ci.conf_dbg_log_enabled) && (WorldState::instance().s().n_detected_objects > 0))
      {
        printf ("DETECT:\n");

#if 0 /* CRIDF2021 */
        char color_s[16];
        if (dbg_color_code==1)
        {
          strncpy(color_s,"RED  :",16);
        }
        else if (dbg_color_code==2)
        {
          strncpy(color_s,"GREEN:",16);
        }
        else
        {
          strncpy(color_s,"?????:",16);
        }
        printf ("%s:\n", color_s);
        printf (" Prel=<%f,%f>\n", dbg_x_rel, dbg_y_rel);
        printf (" Pabs=<%f,%f>\n", dbg_x_abs, dbg_y_abs);
        printf ("\n");
#endif

#if 1 /* DEMO2022 */
        printf (" ID=%d\n", dbg_id_code);
        printf (" Prel=<%f,%f>\n", dbg_x_rel, dbg_y_rel);
        printf (" Pabs=<%f,%f>\n", dbg_x_abs, dbg_y_abs);
        printf ("\n");
#endif
      } /* if(WorldState::instance().s().n_detected_objects > 0) */

      dbg_cnt = 0;
    }
    else
    {
      dbg_cnt++;
    }
#endif /* FIXME : DEBUG : DISPLAY */

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
  return zmq_send (m_pub_socket, buf, len, flags);
}

int CommZmq::recv(void *buf, size_t len, int flags)
{
  if (m_pull_socket==NULL) return -1;
  return zmq_recv (m_pull_socket, buf, len, flags);
}

void CommZmq::send_robot_detection(const char *buf, size_t len)
{
  unsigned short my_message_type;

  my_message_type = 1280; /* FIXME : TODO : use mesage_types.hpp */

  send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
  send(buf, len, 0);
}

