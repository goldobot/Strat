#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>

#define ROBOT_SIM 1

#include "comm_nucleo.hpp"
#include "sim/virtual_robot.hpp"


using namespace goldobot;

DirectUartNucleo DirectUartNucleo::s_instance;


DirectUartNucleo& DirectUartNucleo::instance()
{
  return s_instance;
}

DirectUartNucleo::DirectUartNucleo()
{
  strncpy(m_thread_name,"DirectUartNucleo_sim",sizeof(m_thread_name));

  m_stop_task = false;
  m_task_running = false;

  memset(m_devname, 0, sizeof(m_devname));
  m_uart_fd = -1;

  memset(&m_savetio_remote, 0, sizeof(m_savetio_remote));

  memset(&m_ibuf, 0, sizeof(m_ibuf));
  memset(&m_obuf, 0, sizeof(m_obuf));

  m_payload_byte_cnt = 0;

  m_uart_send_seq = 0;
  memset(&m_send_buf, 0, sizeof(m_send_buf));

  memset(&m_payload_buf, 0, sizeof(m_payload_buf));

  m_read_odo_state = READ_ODO_STATE_INIT;

  m_debug_num_points = 0;
  memset(&m_debug_traj_x_mm, 0, sizeof(m_debug_traj_x_mm));
  memset(&m_debug_traj_y_mm, 0, sizeof(m_debug_traj_y_mm));
}

int DirectUartNucleo::set_interface_attribs(int fd, int speed)
{
  /* bouchon */
  fd = fd;
  speed = speed;

  return 0;
}

int DirectUartNucleo::init(char *uart_dev_name, int speed)
{
  strncpy(m_devname, uart_dev_name, sizeof(m_devname));
  m_devname[sizeof(m_devname)-1] = 0x00;

  /* bouchon */
  speed = speed;

  m_uart_fd = -1;

  return 0;
}

void DirectUartNucleo::taskFunction()
{

  m_task_running = true;

  while(!m_stop_task)
  {

    /* FIXME : TODO : robot simulation */
#if 0
    RobotState::instance().lock();
    RobotState::instance().s().local_ts_ms   = my_time_ms;
    RobotState::instance().s().remote_ts_ms  = cur_odo_time;
    RobotState::instance().s().x_mm          = cur_odo_x_mm;
    RobotState::instance().s().y_mm          = cur_odo_y_mm;
    RobotState::instance().s().theta_deg     = cur_odo_theta_deg;
    RobotState::instance().s().robot_sensors = cur_robot_sensors_32;
    RobotState::instance().release();
#endif

    usleep(10000);

    pthread_yield();
  }

  m_task_running = false;
}

int DirectUartNucleo::send(const unsigned char *msg_buf, size_t msg_len)
{
#if 1 /* FIXME : DEBUG */
  {
    printf("DEBUG: direct_uart_sim: sending %lu bytes:\n", msg_len);
    for (unsigned int i=0; i<msg_len; i++) printf ("%.2x ",msg_buf[i]);
    printf ("\n");
  }
#endif

  VirtualRobot::myself().sim_receive(msg_buf, msg_len);

  return 0;
}

int DirectUartNucleo::recv(unsigned char *msg_buf, size_t msg_len)
{
  /* FIXME : TODO */
  msg_buf = msg_buf;
  msg_len = msg_len;
  return -1; /* not implemented yet */
}

void DirectUartNucleo::exit_thread(int err_code)
{
  while (1) {
    m_stop_task = true;
    pthread_yield();
  }
}




