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
#include "comm_zmq.hpp"
#include "comm_nucleo.hpp"
#include "comm_rplidar.hpp"


using namespace goldobot;
using namespace std;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif


#if 1 /* FIXME : DEBUG */
struct PropulsionTelemetry
{
  int16_t x;//quarters of mm
  int16_t y;
  int16_t yaw;
  int16_t speed;// mm per second
  int16_t yaw_rate;// mradian per second
  int16_t acceleration;
  int16_t angular_acceleration;
  uint16_t left_encoder;
  uint16_t right_encoder;
  int8_t left_pwm;// percents
  int8_t right_pwm;
  uint8_t state;
  uint8_t error;
};

#endif

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
  m_comm_uart_pub_socket = NULL;
  m_comm_uart_pull_socket = NULL;
}

int CommZmq::init(int port_nb, int comm_uart_port_nb)
{
  int rc;
  char char_buff[64];

  m_zmq_context = zmq_init(1);

  m_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
  if (m_pub_socket<0) {
    printf ("RPLIDAR : cannot create ZMQ_PUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", port_nb);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_bind(m_pub_socket, char_buff);
  if (rc<0) {
    printf ("RPLIDAR : cannot bind ZMQ_PUB socket\n");
    return -1;
  }

  m_pull_socket = zmq_socket(m_zmq_context, ZMQ_SUB);
  if (m_pull_socket<0) {
    printf ("RPLIDAR : cannot create ZMQ_SUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://*:%d", port_nb+1);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_bind(m_pull_socket, char_buff);
  if (rc<0) {
    printf ("RPLIDAR : cannot bind ZMQ_SUB socket\n");
    return -1;
  }
  zmq_setsockopt(m_pull_socket,ZMQ_SUBSCRIBE, "", 0);


  printf("  ZMQ DEBUG: comm_uart_port_nb = %d\n", comm_uart_port_nb);

  m_comm_uart_pull_socket = zmq_socket(m_zmq_context, ZMQ_SUB);
  if (m_comm_uart_pull_socket<0) {
    printf ("RPLIDAR : cannot create comm_uart ZMQ_SUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://127.0.0.1:%d", comm_uart_port_nb);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_connect(m_comm_uart_pull_socket, char_buff);
  if (rc<0) {
    printf ("RPLIDAR : cannot connect comm_uart ZMQ_SUB socket\n");
    return -1;
  }
  zmq_setsockopt(m_comm_uart_pull_socket,ZMQ_SUBSCRIBE, "", 0);

  m_comm_uart_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);
  if (m_comm_uart_pub_socket<0) {
    printf ("RPLIDAR : cannot create comm_uart ZMQ_PUB socket\n");
    return -1;
  }

  sprintf(char_buff, "tcp://127.0.0.1:%d", comm_uart_port_nb+1);
  //printf("  ZMQ DEBUG: char_buff = %s\n", char_buff);
  rc = zmq_connect(m_comm_uart_pub_socket, char_buff);
  if (rc<0) {
    printf ("RPLIDAR : cannot connect comm_uart ZMQ_PUB socket\n");
    return -1;
  }


  return 0;
}

void CommZmq::taskFunction()
{
  goldo_conf_info_t& ci = GoldoConf::instance().c();
  struct timespec curr_tp;
  int curr_time_ms = 0;
  int old_time_ms = 0;
  bool have_msg = false;
  bool is_comm_uart = false;

  unsigned char buff[1024];
  size_t bytes_read = 0;

  zmq_pollitem_t poll_items[2];

  poll_items[0].socket = m_pull_socket;
  poll_items[0].fd = 0;
  poll_items[0].events = ZMQ_POLLIN;
  poll_items[1].socket = m_comm_uart_pull_socket;
  poll_items[1].fd = 0;
  poll_items[1].events = ZMQ_POLLIN;

  m_task_running = true;

  FILE *dbg_log_fd   = NULL;
  FILE *dbg_pos_fd   = NULL;
  FILE *dbg_x_fd     = NULL;
  FILE *dbg_y_fd     = NULL;
  FILE *dbg_theta_fd = NULL;
  if (ci.conf_dbg_log_enabled)
  {
    dbg_log_fd   = fopen("log/comm_dbg.txt", "wt");
    dbg_pos_fd   = fopen("log/dbg_pos.txt", "wt");
    dbg_x_fd     = fopen("log/dbg_x.txt", "wt");
    dbg_y_fd     = fopen("log/dbg_y.txt", "wt");
    dbg_theta_fd = fopen("log/dbg_theta.txt", "wt");

    if ((dbg_log_fd==NULL)) 
    {
      fprintf(stderr,"ERROR : comm_zmq : cannot create log files\n");
      ci.conf_dbg_log_enabled = false;
    }
  }

  while(!m_stop_task)
  {
    zmq_poll (poll_items, 2, 100);

    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if (curr_time_ms > (old_time_ms + 100)) {
      /* FIXME : TODO : send some heartbeat message? */

      old_time_ms = curr_time_ms;
    }

    bytes_read = 0;

    if(poll_items[0].revents && ZMQ_POLLIN)
    {            
      int64_t more=1;
      size_t more_size = sizeof(more);
      while(more)
      {
        bytes_read += zmq_recv(m_pull_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);           
        zmq_getsockopt(m_pull_socket, ZMQ_RCVMORE, &more, &more_size);
      }
      buff[bytes_read] = 0;
      have_msg = true;
      is_comm_uart = false;
    }

    if(poll_items[1].revents && ZMQ_POLLIN)
    {            
      int64_t more=1;
      size_t more_size = sizeof(more);
      while(more)
      {
        bytes_read += zmq_recv(m_comm_uart_pull_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);           
        zmq_getsockopt(m_comm_uart_pull_socket, ZMQ_RCVMORE, &more, &more_size);
      }
      buff[bytes_read] = 0;
      have_msg = true;
      is_comm_uart = true;
    }

    if(have_msg)
    {
      /*while (bytes_read!=0)*/
      {
        uint16_t message_type = 0;
        memcpy (&message_type, buff, sizeof(message_type));

        if (ci.conf_dbg_log_enabled)
        {
          if(is_comm_uart)
            dbg_dump_msg(dbg_log_fd, "COMM_UART : ", buff, bytes_read);
          else
            dbg_dump_msg(dbg_log_fd, "GOLDO_IHM : ", buff, bytes_read);
        }

        /* FIXME : TODO : import message_types.h into the project */
        switch (message_type) {
        case 1024: /* RplidarStart                   */
          printf ("  ZMQ DEBUG: RplidarStart\n");
          CommRplidar::instance().start_scan();
          break;
        case 1025: /* RplidarStop                    */
          printf ("  ZMQ DEBUG: RplidarStop\n");
          CommRplidar::instance().stop_scan();
          break;
        case 8:   /* PropulsionTelemetry             */
          int log_time_ms = 0;
          clock_gettime(1, &curr_tp);
          log_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;
          unsigned char *dbg_p = buff+2;
          PropulsionTelemetry *ptm = (PropulsionTelemetry *) dbg_p;
          double dbg_x_mm = (double) ptm->x / 4.0;
          double dbg_y_mm = (double) ptm->y / 4.0;
          double dbg_theta_deg = 180.0 * ptm->yaw / 32767.0;
          double l_odo_x_mm      = (double)RobotState::instance().s().x_mm;
          double l_odo_y_mm      = (double)RobotState::instance().s().y_mm;
          double l_odo_theta_deg = (double)RobotState::instance().s().theta_deg;

          if (ci.conf_dbg_log_enabled)
          {
            fprintf(dbg_pos_fd, "TS %d\n", log_time_ms);
            fprintf(dbg_pos_fd, "DIRECT %f %f %f\n",
                    l_odo_x_mm, l_odo_y_mm, l_odo_theta_deg);
            fprintf(dbg_pos_fd, "COMM %f %f %f\n",
                    dbg_x_mm, dbg_y_mm, dbg_theta_deg);
            fprintf(dbg_x_fd, "%d %f\n", log_time_ms, 
                    dbg_x_mm - l_odo_x_mm);
            fprintf(dbg_y_fd, "%d %f\n", log_time_ms, 
                    dbg_y_mm - l_odo_y_mm);
            fprintf(dbg_theta_fd, "%d %f\n", log_time_ms, 
                    dbg_theta_deg - l_odo_theta_deg);
          }

          if (!ci.conf_direct_uart_nucleo_enabled)
          {
            RobotState::instance().lock();
            RobotState::instance().s().x_mm          = dbg_x_mm;
            RobotState::instance().s().y_mm          = dbg_y_mm;
            RobotState::instance().s().theta_deg     = dbg_theta_deg;
            RobotState::instance().release();
          }

          break;
        }
      }
    }
    have_msg = false;
    is_comm_uart = false;

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  }

  zmq_term(m_zmq_context);

  if (ci.conf_dbg_log_enabled)
  {
    fclose(dbg_log_fd);
    fclose(dbg_pos_fd);
    fclose(dbg_x_fd);
    fclose(dbg_y_fd);
    fclose(dbg_theta_fd);
  }

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

void CommZmq::dbg_dump_msg(
  FILE *dbg_log_fd, const char *prefix, unsigned char *buff, size_t len)
{
  struct timespec curr_tp;
  int log_time_ms = 0;

  clock_gettime(1, &curr_tp);

  log_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

  fprintf(dbg_log_fd, "%d : ", log_time_ms);

  fprintf(dbg_log_fd, "%s : ", prefix);

  for (int i=0; i<(int)len; i++) 
    fprintf(dbg_log_fd, "%.2x ", (int)buff[i]);

  fprintf(dbg_log_fd, "\n");
}
