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
}

int CommZmq::init(int port_nb)
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

  return 0;
}

void CommZmq::taskFunction()
{
  struct timespec curr_tp;
  int curr_time_ms = 0;
  int old_time_ms = 0;


  zmq_pollitem_t poll_items[1];

  poll_items[0].socket = m_pull_socket;
  poll_items[0].fd = 0;
  poll_items[0].events = ZMQ_POLLIN;

  m_task_running = true;

  while(!m_stop_task)
  {
    zmq_poll (poll_items, 1, 100);

    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if (curr_time_ms > (old_time_ms + 100)) {
      /* FIXME : TODO : send some heartbeat message? */

      old_time_ms = curr_time_ms;
    }

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
  return zmq_send (m_pub_socket, buf, len, flags);
}

int CommZmq::recv(void *buf, size_t len, int flags)
{
  if (m_pull_socket==NULL) return -1;
  return zmq_recv (m_pull_socket, buf, len, flags);
}

