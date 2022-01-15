#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include <unistd.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "comm_rplidar.hpp"
#include "goldo_conf.hpp"
#include "detect/lidar_detect.hpp"

#pragma GCC diagnostic ignored "-Wstringop-truncation"

using namespace goldobot;

using namespace rp::standalone::rplidar;


CommRplidar CommRplidar::s_instance;

CommRplidar& CommRplidar::instance()
{
  return s_instance;
}

CommRplidar::CommRplidar()
{
  int i;

  strncpy(m_thread_name,"CommRplidar_sim",sizeof(m_thread_name));

  m_stop_task = false;
  m_task_running = false;
  m_scanning = false;

  memset (m_rplidar_dev_str, 0, sizeof(m_rplidar_dev_str));

  m_baudrate = 0;

  for (i=0; i<720; i++)
  {
    m_x[i] = 0.0;
    m_y[i] = 0.0;
  }

  m_drv = NULL;

  m_viewer_sock = -1;
  memset (m_viewer_addr_str, 0, sizeof(m_viewer_addr_str));
  memset (&m_viewer_saddr, 0, sizeof(m_viewer_saddr));
  memset (m_viewer_send_buf, 0, sizeof(m_viewer_send_buf));
}

int CommRplidar::init(char* rplidar_dev, int baudrate)
{
  int i;

  strncpy(m_rplidar_dev_str, rplidar_dev, sizeof(m_rplidar_dev_str));

  for (i=0; i<M_NB_POINTS; i++)
  {
    m_x[i] = 0.0;
    m_y[i] = 0.0;
  }

  m_baudrate = baudrate;

  m_viewer_sock = -1;
  memset (&m_viewer_saddr, 0, sizeof(m_viewer_saddr));
  memset (m_viewer_send_buf, 0, sizeof(m_viewer_send_buf));

  m_drv = NULL;

  return 0;
}

int CommRplidar::init_viewer_sock(char *viewer_address_str)
{
#ifndef WIN32
  unsigned int ab0, ab1, ab2, ab3;

  strncpy(m_viewer_addr_str, viewer_address_str, sizeof(m_viewer_addr_str));

  if (sscanf (viewer_address_str, "%d.%d.%d.%d", 
              &ab3, &ab2, &ab1, &ab0) != 4) {
    printf(" error : cannot parse viewer addr (%s)\n", viewer_address_str);
    return -1;
  }

  m_viewer_sock = socket (AF_INET, SOCK_DGRAM, 0);

  m_viewer_saddr.sin_family= AF_INET;
  m_viewer_saddr.sin_port= htons(1413);
  m_viewer_saddr.sin_addr.s_addr= htonl((ab3<<24)|(ab2<<16)|(ab1<<8)|(ab0));
#else
  viewer_address_str = viewer_address_str;
#endif

  return 0;
}

void CommRplidar::start_scan()
{
  if (m_scanning) return;

  /* FIXME : TODO */

  m_scanning = true;
}

void CommRplidar::stop_scan()
{
  m_scanning = false;

  /* FIXME : TODO */
}

int CommRplidar::send_to_viewer()
{
#ifndef WIN32
  unsigned int *cur_ptr_w = (unsigned int *)((void *)(&m_viewer_send_buf[0]));
  int bytes_to_send = 0;
  int l_odo_theta_deg_0_01 = 0;

  RobotState::instance().lock();
  int l_odo_x_mm      = RobotState::instance().s().x_mm;
  int l_odo_y_mm      = RobotState::instance().s().y_mm;
  int l_odo_theta_deg = RobotState::instance().s().theta_deg;
  RobotState::instance().release();

  /* header tlv */
  *cur_ptr_w = htonl(0x31337000);
  cur_ptr_w++; bytes_to_send+=4;
  *cur_ptr_w = htonl(0); /* longueur totale : recalcule + bas */
  cur_ptr_w++; bytes_to_send+=4;
  /* on reserve l'id 0x31337001 pour "header etendu" */
  /* odometrie */
  *cur_ptr_w = htonl(0x31337002);
  cur_ptr_w++; bytes_to_send+=4;
  *cur_ptr_w = htonl(l_odo_x_mm);
  cur_ptr_w++; bytes_to_send+=4;
  *cur_ptr_w = htonl(l_odo_y_mm);
  cur_ptr_w++; bytes_to_send+=4;
  l_odo_theta_deg_0_01 = l_odo_theta_deg*100.0;
  *cur_ptr_w = htonl(l_odo_theta_deg_0_01);
  cur_ptr_w++; bytes_to_send+=4;
  /* points du scan lidar */
  *cur_ptr_w = htonl(0x31337003);
  cur_ptr_w++; bytes_to_send+=4;
  for (int i=0; i<720; i++) {
    int my_x_int = m_x[i];
    int my_y_int = m_y[i];

    *cur_ptr_w = (unsigned int) my_x_int;
    cur_ptr_w++; bytes_to_send+=4;
    *cur_ptr_w = (unsigned int) my_y_int;
    cur_ptr_w++; bytes_to_send+=4;
  }
  /* marqueur de fin */
  *cur_ptr_w = htonl(0x313370ff);
  cur_ptr_w++; bytes_to_send+=4;

  cur_ptr_w = (unsigned int *)((void *)(&m_viewer_send_buf[0]));
  cur_ptr_w[1] = bytes_to_send;

  int ret = sendto(m_viewer_sock, (void *)m_viewer_send_buf, bytes_to_send,0, 
                   (const sockaddr *) &m_viewer_saddr, 
                   sizeof (struct sockaddr_in));
  if (ret!=bytes_to_send) {
    printf ("sendto() failed\n");
    return -1;
  }
#endif

  return 0;
}

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

void CommRplidar::taskFunction()
{

  m_task_running = true;

  while(!m_stop_task)
  {
    if (!m_scanning)
    {
#ifndef WIN32
      usleep(10000);
#else
      Sleep(10);
#endif

#ifndef WIN32
      pthread_yield();
#else
      sched_yield();
#endif
      continue;
    }

    /* FIXME : TODO */

#ifndef WIN32
    usleep(10000);
#else
    Sleep(10);
#endif

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  } /* while(!m_stop_task) */


  m_task_running = false;
}


