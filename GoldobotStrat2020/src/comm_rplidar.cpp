#ifdef WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "comm_rplidar.hpp"
#include "goldo_conf.hpp"
#include "detect/lidar_detect.hpp"


using namespace goldobot;

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv);
#if 1 /* FIXME : DEBUG : HACK GOLDO : minimalist obstacle detection */ 
void set_obstacle_gpio(bool obstacle_detect);
#endif


CommRplidar CommRplidar::s_instance;

CommRplidar& CommRplidar::instance()
{
  return s_instance;
}

CommRplidar::CommRplidar()
{
  int i;

  strncpy(m_thread_name,"CommRplidar",sizeof(m_thread_name));

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

  // create the driver instance
  m_drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

  if (!m_drv) {
    fprintf(stderr, "RPlidarDriver::CreateDriver() error\n");
    return -1;
  }

  return 0;
}

int CommRplidar::init_viewer_sock(char *viewer_address_str)
{
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

  return 0;
}

void CommRplidar::start_scan()
{
  if ((m_drv==NULL) || (m_scanning)) return;

  m_drv->startMotor();
  m_drv->startScan();
  m_scanning = true;
}

void CommRplidar::stop_scan()
{
  m_scanning = false;
  if (m_drv!=NULL)
  {
    m_drv->stop();
    m_drv->stopMotor();
  }
}

int CommRplidar::send_to_viewer()
{
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

  return 0;
}

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

void CommRplidar::taskFunction()
{
  u_result     op_result;

  goldo_conf_info_t& ci = GoldoConf::instance().c();
  double l_theta_correction = ci.conf_theta_correction_deg*M_PI/180.0f;
  double l_rho_correction_factor = ci.conf_rho_correction_factor;

  // make connection...
  if (IS_FAIL(m_drv->connect(m_rplidar_dev_str, m_baudrate))) {
    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , m_rplidar_dev_str);
    goto on_finished;
  }

  rplidar_response_device_info_t devinfo;

  // retrieve the device info
  op_result = m_drv->getDeviceInfo(devinfo);

  if (IS_FAIL(op_result)) {
    fprintf(stderr, "Error, cannot get device info.\n");
    goto on_finished;
  }

#if 1 /* FIXME : DEBUG */
  // print out the device serial number, firmware and hardware version number
  printf("RPLIDAR S/N: ");
  for (int pos = 0; pos < 16 ;++pos) {
    printf("%02X", devinfo.serialnum[pos]);
  }

  printf("\n"
         "Firmware Ver: %d.%02d\n"
         "Hardware Rev: %d\n"
         , devinfo.firmware_version>>8
         , devinfo.firmware_version & 0xFF
         , (int)devinfo.hardware_version);
#endif /* FIXME : DEBUG */

  // check health...
  if (!checkRPLIDARHealth(m_drv)) {
    fprintf(stderr, "Error, rplidar health bad!\n");
    goto on_finished;
  }

  m_task_running = true;

  //start_scan();

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

    bool obstacle_detect = false;

    rplidar_response_measurement_node_t nodes[M_NB_POINTS];
    size_t count = _countof(nodes);

    for (int pos = 0; pos < M_NB_POINTS; ++pos) {
      m_x[pos] = 0.0;
      m_y[pos] = 0.0;
    }

    LidarDetect::instance().clearSlots();


    op_result = m_drv->grabScanData(nodes, count);

    if (IS_OK(op_result)) 
    {
      m_drv->ascendScanData(nodes, count);

      for (int pos = 0; pos < (int)count ; ++pos) 
      {
        double my_theta = ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f)*(2.0f*M_PI/360.0f) + l_theta_correction;
        my_theta = -my_theta; /* FIXME : TODO : explication? (WTF?!) */
        double my_R = l_rho_correction_factor * nodes[pos].distance_q2/4.0f;

        RobotState::instance().lock();
        int l_odo_x_mm         = RobotState::instance().s().x_mm;
        int l_odo_y_mm         = RobotState::instance().s().y_mm;
        double l_odo_theta_deg = RobotState::instance().s().theta_deg;
        bool l_forward_move    = RobotState::instance().s().forward_move;
        RobotState::instance().release();

        if ((my_R < 1.0f) || (my_R > 3000.0f)) my_R = 0.0f;

        m_x[pos] = my_R * cos (my_theta);
        m_y[pos] = my_R * sin (my_theta);

        double l_odo_theta_rad = l_odo_theta_deg*M_PI/180.0;

        double my_abs_x = my_R * cos (my_theta + l_odo_theta_rad) + l_odo_x_mm;
        double my_abs_y = my_R * sin (my_theta + l_odo_theta_rad) + l_odo_y_mm;

        /* minimalist obstacle detection */ 
        /* check si proche des bords anterieur ou posterieurs du robot */ 
        if(
           (
            ((m_x[pos]>90.0)&& (m_x[pos]<300.0)&& (l_forward_move)) 
            ||
            ((m_x[pos]<-90.0)&& (m_x[pos]>-300.0)&& (!l_forward_move))
           ) 
           && 
           (
            ((m_y[pos]>-140.0) && (m_y[pos]<140.0))
           )
          ) 
        {
          //printf("GOLDO my_theta:%f my_R:%f m_x[pos]:%f m_y[pos]:%f\n", my_theta, my_R, m_x[pos], m_y[pos]);

          /* check si a l'interieur du terain de jeu */ 
          if ((my_abs_x >   100.0) && (my_abs_x < 1400.0) && 
              (my_abs_y > -1400.0) && (my_abs_y < 1400.0)) 
          {
            obstacle_detect = true;
          }
        }

        struct timespec my_tp;
        clock_gettime(1, &my_tp);
        int my_thread_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

        /* envoi des echantillons au tracker d'adversaire */ 
        if ((my_abs_x >   100.0) && (my_abs_x < 1900.0) && 
            (my_abs_y > -1400.0) && (my_abs_y < 1400.0) && 
            (my_R > 100.0) ) {
          LidarDetect::instance().recordNewLidarSample(my_thread_time_ms, my_abs_x, my_abs_y);
        }
      } /* for (int pos = 0; pos < (int)count ; ++pos) */

      LidarDetect::instance().updateDetection();

      LidarDetect::instance().sendDetected();

      send_to_viewer();

#if 1 /* FIXME : DEBUG : HACK GOLDO : minimalist obstacle detection */ 
      set_obstacle_gpio(obstacle_detect);
#endif

    } /* if (IS_OK(op_result = m_drv->grabScanData(nodes, count))) */

#ifndef WIN32
    pthread_yield();
#else
    sched_yield();
#endif
  } /* while(!m_stop_task) */

//stop_device:
  stop_scan();
  // done!
on_finished:
  RPlidarDriver::DisposeDriver(m_drv);

  m_task_running = false;
}


#if 1 /* FIXME : DEBUG : HACK GOLDO : minimalist obstacle detection */ 
void set_obstacle_gpio(bool obstacle_detect)
{
  int goldo_detect_fd;
  char write_buf[8];
  int res;
  goldo_detect_fd = open ("/sys/class/gpio/gpio21/value", O_RDWR);
  if (goldo_detect_fd<0) {
    printf ("error opening gpio\n");
    return;
  }
  if (obstacle_detect)
    write_buf[0] = '1'; 
  else 
    write_buf[0] = '0';
  write_buf[1] = '\0';
  res = write (goldo_detect_fd,write_buf,1);
  if (res<0) {
    printf ("error writting gpio value\n");
  }
  close (goldo_detect_fd);
}
#endif

/* FIXME : TODO : horrible!.. */
bool checkRPLIDARHealth(RPlidarDriver * drv)
{
  u_result     op_result;
  rplidar_response_device_health_t healthinfo;

  op_result = drv->getHealth(healthinfo);
  if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
#if 0 /* FIXME : DEBUG : HACK GOLDO ++ */
    printf("RPLidar health status : %d\n", healthinfo.status);
#endif /* FIXME : DEBUG : HACK GOLDO -- */
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
      fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
      // enable the following code if you want rplidar to be reboot by software
      // drv->reset();
      return false;
    } else {
      return true;
    }

  } else {
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
    return false;
  }
}
