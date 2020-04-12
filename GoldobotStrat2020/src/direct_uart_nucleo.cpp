#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>

#include "direct_uart_nucleo.hpp"


using namespace goldobot;

DirectUartNucleo DirectUartNucleo::s_instance;


DirectUartNucleo& DirectUartNucleo::instance()
{
  return s_instance;
}

DirectUartNucleo::DirectUartNucleo()
{
  strncpy(m_thread_name,"DirectUartNucleo",sizeof(m_thread_name));

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
  struct termios tty;

  if (tcgetattr(fd, &tty) < 0) {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }

/* FIXME : TODO : chose correct setting */
#if 1 /* FIXME : DEBUG : Thomas style */
  cfsetospeed(&tty, (speed_t)speed);
  cfsetispeed(&tty, (speed_t)speed);

  tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;         /* 8-bit characters */
  tty.c_cflag &= ~PARENB;     /* no parity bit */
  tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

  /* setup for non-canonical mode */
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  /* fetch bytes as they become available */
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

#else /* FIXME : DEBUG : Goldo style */
  memset(&tty, 0, sizeof(tty));

  tty.c_cflag = CREAD | HUPCL | speed;

  tty.c_cflag |= CLOCAL;

  /* no parity bit */
  //tty.c_cflag &= ~PARENB;

  /* 8-bit characters */
  //tty.c_cflag |= CS8;

  /* only need 1 stop bit */
  //tty.c_cflag &= ~CSTOPB;

  /* no hardware flowcontrol */
  //tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

#endif

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }

  return 0;
}

int DirectUartNucleo::init(char *uart_dev_name, int speed)
{
  int fd;

  strncpy(m_devname, uart_dev_name, sizeof(m_devname));
  m_devname[sizeof(m_devname)-1] = 0x00;

/* FIXME : TODO : chose correct setting */
  fd = open(m_devname, O_RDWR | O_NOCTTY | O_SYNC); /* Thomas style */
  //fd = open(m_devname, O_RDWR | O_NDELAY); /* Goldo style */
  if (fd<0) {
    printf("Cannot open uart device (%s)\n", m_devname);
    return -1;
  }

  if (set_interface_attribs(fd, B115200) != 0) {
    return -1;
  }

  m_uart_fd = fd;

  return 0;
}

#define READ_SHORT_DBG(_lvalue, _offset) do { \
  cp = &m_payload_buf[_offset];               \
  swp = (unsigned short *) cp;                \
  _lvalue = *swp;                             \
} while (0)

void DirectUartNucleo::taskFunction()
{
  const int PAYLOAD_BUF_SZ = sizeof(m_payload_buf);

  fd_set infds;
  int maxfd, n;

  unsigned int cur_odo_time;

  double cur_odo_x_mm;
  double cur_odo_y_mm;
  double cur_odo_theta_deg;
  short cur_odo_x_16;
  short cur_odo_y_16;
  int cur_odo_theta_32;
  unsigned int cur_robot_sensors_32;

  unsigned int *wp;
  unsigned short *swp;
  unsigned char *cp;


  if (m_uart_fd<0) {
    fprintf(stderr, "ERROR: DirectUartNucleo::taskFunction(): m_uart_fd=%d\n", m_uart_fd);
    exit_thread(1);
  }

  maxfd = m_uart_fd;
  maxfd++;

  m_read_odo_state = READ_ODO_STATE_INIT;

  m_payload_byte_cnt = 0;

  m_task_running = true;

  while(!m_stop_task)
  {
    FD_ZERO(&infds);
    FD_SET(m_uart_fd, &infds);

    /* FIXME : TODO : define select() timeout! */
    if (select(maxfd, &infds, NULL, NULL, NULL) < 0)
    {
      fprintf(stderr, "ERROR: DirectUartNucleo::taskFunction(): select() failed, errno=%d\n", errno);
      /* FIXME : TODO : necessary? */
      //exit_thread(1);
    }

    if (FD_ISSET(m_uart_fd, &infds))
    {
      if ((n = read(m_uart_fd, m_ibuf, 1)) < 0) {
        fprintf(stderr, "ERROR: DirectUartNucleo::taskFunction(): read() failed, errno=%d\n", errno);
        /* FIXME : TODO : necessary? */
        //exit_thread(1);
      }

      switch (m_read_odo_state) {
      case READ_ODO_STATE_INIT:
      case READ_ODO_STATE_HEAD0:
        m_payload_byte_cnt = 0;
        memset(m_payload_buf, 0, PAYLOAD_BUF_SZ);
        if (m_ibuf[0]==0x0a)
          m_read_odo_state = READ_ODO_STATE_HEAD1;
        else
          m_read_odo_state = READ_ODO_STATE_INIT;
        break;
      case READ_ODO_STATE_HEAD1:
        if (m_ibuf[0]==0x35)
          m_read_odo_state = READ_ODO_STATE_HEAD2;
        else
          m_read_odo_state = READ_ODO_STATE_INIT;
        break;
      case READ_ODO_STATE_HEAD2:
        if (m_ibuf[0]==0x0a)
          m_read_odo_state = READ_ODO_STATE_HEAD3_ODO;
        else if (m_ibuf[0]==0x00)
          m_read_odo_state = READ_ODO_STATE_HEAD3_DBG;
        else
          m_read_odo_state = READ_ODO_STATE_INIT;
        break;

      case READ_ODO_STATE_HEAD3_ODO:
        if (m_ibuf[0]==0x35)
          m_read_odo_state = READ_ODO_STATE_TS0;
        else
          m_read_odo_state = READ_ODO_STATE_INIT;
        break;
      case READ_ODO_STATE_TS0:
        wp = &cur_odo_time;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_TS1;
        cp[0] = m_ibuf[0];
        break;

      case READ_ODO_STATE_TS1:
        wp = &cur_odo_time;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_TS2;
        cp[1] = m_ibuf[0];
        break;
      case READ_ODO_STATE_TS2:
        wp = &cur_odo_time;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_TS3;
        cp[2] = m_ibuf[0];
        break;
      case READ_ODO_STATE_TS3:
        wp = &cur_odo_time;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_X0;
        cp[3] = m_ibuf[0];
        break;
      case READ_ODO_STATE_X0:
        swp = (unsigned short *) &cur_odo_x_16;
        cp = (unsigned char *) swp;
        m_read_odo_state = READ_ODO_STATE_X1;
        cp[0] = m_ibuf[0];
        break;
      case READ_ODO_STATE_X1:
        swp = (unsigned short *) &cur_odo_x_16;
        cp = (unsigned char *) swp;
        m_read_odo_state = READ_ODO_STATE_Y0;
        cp[1] = m_ibuf[0];
        break;
      case READ_ODO_STATE_Y0:
        swp = (unsigned short *) &cur_odo_y_16;
        cp = (unsigned char *) swp;
        m_read_odo_state = READ_ODO_STATE_Y1;
        cp[0] = m_ibuf[0];
        break;
      case READ_ODO_STATE_Y1:
        swp = (unsigned short *) &cur_odo_y_16;
        cp = (unsigned char *) swp;
        m_read_odo_state = READ_ODO_STATE_THETA0;
        cp[1] = m_ibuf[0];
        break;
      case READ_ODO_STATE_THETA0:
        wp = (unsigned int *) &cur_odo_theta_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_THETA1;
        cp[0] = m_ibuf[0];
        break;
      case READ_ODO_STATE_THETA1:
        wp = (unsigned int *) &cur_odo_theta_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_THETA2;
        cp[1] = m_ibuf[0];
        break;
      case READ_ODO_STATE_THETA2:
        wp = (unsigned int *) &cur_odo_theta_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_THETA3;
        cp[2] = m_ibuf[0];
        break;
      case READ_ODO_STATE_THETA3:
        wp = (unsigned int *) &cur_odo_theta_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_SENSORS0;
        cp[3] = m_ibuf[0];
        break;
      case READ_ODO_STATE_SENSORS0:
        wp = (unsigned int *) &cur_robot_sensors_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_SENSORS1;
        cp[0] = m_ibuf[0];
        break;
      case READ_ODO_STATE_SENSORS1:
        wp = (unsigned int *) &cur_robot_sensors_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_SENSORS2;
        cp[1] = m_ibuf[0];
        break;
      case READ_ODO_STATE_SENSORS2:
        wp = (unsigned int *) &cur_robot_sensors_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_SENSORS3;
        cp[2] = m_ibuf[0];
        break;
      case READ_ODO_STATE_SENSORS3:
        wp = (unsigned int *) &cur_robot_sensors_32;
        cp = (unsigned char *) wp;
        m_read_odo_state = READ_ODO_STATE_INIT;
        cp[3] = m_ibuf[0];

        cur_odo_x_mm = cur_odo_x_16;
        cur_odo_y_mm = cur_odo_y_16;
        cur_odo_theta_deg = cur_odo_theta_32/1000.0;

#if 0 /* FIXME : DEBUG */
        printf ("ts = %u ; pos = < %f , %f > ; theta = %f\n",
                cur_odo_time, 
                cur_odo_x_mm, cur_odo_y_mm, 
                cur_odo_theta_deg);
#endif

#if 0 /* FIXME : DEBUG */
        if (dump_file) {
          fprintf (dump_file, 
                   "ts = %u ; pos = < %f , %f > ; theta = %f\n",
                   cur_odo_time, 
                   cur_odo_x_mm, cur_odo_y_mm, 
                   cur_odo_theta_deg);
        }
#endif

        struct timespec my_tp;
        unsigned int my_time_ms;

        clock_gettime(1, &my_tp);

        my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

#if 0 /* FIXME : DEBUG */
        printf ("my_time_ms = %u\n\n", my_time_ms);
#endif
#if 0 /* FIXME : DEBUG */
        if (dump_file) {
          fprintf (dump_file, "my_time_ms = %u\n\n", my_time_ms);
        }
#endif

        OdometryState::instance().lock();
        OdometryState::instance().m_local_ts_ms   = my_time_ms;
        OdometryState::instance().m_remote_ts_ms  = cur_odo_time;
        OdometryState::instance().m_x_mm          = cur_odo_x_mm;
        OdometryState::instance().m_y_mm          = cur_odo_y_mm;
        OdometryState::instance().m_theta_deg     = cur_odo_theta_deg;
        OdometryState::instance().m_robot_sensors = cur_robot_sensors_32;
        OdometryState::instance().release();

        break;

      case READ_ODO_STATE_HEAD3_DBG:
        if ((m_ibuf[0]>0x00) && (m_ibuf[0]<=0x10))
        {
          m_read_odo_state = READ_ODO_STATE_PAYLOAD_DBG;
          m_debug_num_points = m_ibuf[0];
        }
        else
        {
          m_read_odo_state = READ_ODO_STATE_INIT;
        }
        break;
      case READ_ODO_STATE_PAYLOAD_DBG:
        m_payload_buf[m_payload_byte_cnt] = m_ibuf[0];
        m_payload_byte_cnt++;
        if (m_payload_byte_cnt<(m_debug_num_points*4))
        {
          m_read_odo_state = READ_ODO_STATE_PAYLOAD_DBG;
        }
        else
        {
          int i;

          for (i=0; i<m_debug_num_points; i++) {
            READ_SHORT_DBG(m_debug_traj_x_mm[i], (4*i));
            READ_SHORT_DBG(m_debug_traj_y_mm[i], (4*i+2));
          }

          printf (" debug_traj (%d) : \n", m_debug_num_points);
          for (i=0; i<m_debug_num_points; i++) {
            printf ("  (%d,%d)\n", m_debug_traj_x_mm[i], m_debug_traj_y_mm[i]);
          }

          m_read_odo_state = READ_ODO_STATE_INIT;
          m_payload_byte_cnt = 0;
        }
        break;

      } /* switch (m_read_odo_state) */

    } /* if (FD_ISSET(m_uart_fd, &infds)) */

    pthread_yield();
  }

  m_task_running = false;
}

int DirectUartNucleo::send(const void *msg_buf, size_t msg_len)
{
  unsigned char *pc = NULL;
  unsigned int  *pw = NULL;
  int res;

  if ((msg_len<0) || (msg_len>(SEND_BUF_SZ-10)) || (msg_buf==NULL)) {
    return -1;
  }

  memset(m_send_buf, 0, SEND_BUF_SZ);

  m_send_buf[0] = 0x55;
  m_send_buf[1] = 0x24;
  m_send_buf[2] = 0x00;
  m_send_buf[3] = (unsigned char) msg_len + 8;

  pw = (unsigned int *) &m_send_buf[4];
  *pw = m_uart_send_seq;
  m_uart_send_seq++;

  pc = (unsigned char *) &m_send_buf[8];
  memcpy (pc, msg_buf, msg_len);

#if 1 /* FIXME : DEBUG */
  {
    int i;
    int dbg_len = msg_len + 8;
    printf("DEBUG: direct_uart: sending %d bytes:\n", dbg_len);
    for (i=0; i<dbg_len; i++) printf ("%.2x ",m_send_buf[i]);
    printf ("\n");
  }
#endif

  if ((res = write(m_uart_fd, m_send_buf, SEND_BUF_SZ)) < 0) {
    fprintf(stderr, "ERROR: write(fd=%d) failed, "
            "errno=%d\n", m_uart_fd, errno);
    return -1;
  }

  return 0;
}

int DirectUartNucleo::recv(void *msg_buf, size_t msg_len)
{
  /* FIXME : TODO */
  return -1; /* not implemented yet */
}

void DirectUartNucleo::exit_thread(int err_code)
{
  while (1) {
    m_stop_task = true;
    pthread_yield();
  }
}




