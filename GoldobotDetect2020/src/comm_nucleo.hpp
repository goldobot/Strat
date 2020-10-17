#pragma once
#include <cstdint>
#include <cstddef>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#ifndef WIN32
#include <termios.h>
#endif
#include <unistd.h>

#include "goldo_thread.hpp"
#include "robot_state.hpp"

namespace goldobot
{
  typedef enum _read_odo_state {
    READ_ODO_STATE_INIT,
    READ_ODO_STATE_HEAD0,

    READ_ODO_STATE_HEAD1,
    READ_ODO_STATE_HEAD2,
    READ_ODO_STATE_HEAD3_ODO,
    READ_ODO_STATE_HEAD3_DBG,

    READ_ODO_STATE_TS0,
    READ_ODO_STATE_TS1,
    READ_ODO_STATE_TS2,
    READ_ODO_STATE_TS3,

    READ_ODO_STATE_X0,
    READ_ODO_STATE_X1,

    READ_ODO_STATE_Y0,
    READ_ODO_STATE_Y1,

    READ_ODO_STATE_THETA0,
    READ_ODO_STATE_THETA1,
    READ_ODO_STATE_THETA2,
    READ_ODO_STATE_THETA3,

    READ_ODO_STATE_SENSORS0,
    READ_ODO_STATE_SENSORS1,
    READ_ODO_STATE_SENSORS2,
    READ_ODO_STATE_SENSORS3,

    READ_ODO_STATE_PAYLOAD_DBG,
  } read_odo_state_t;


  class DirectUartNucleo : public GoldoThread
  {
  public:
    static DirectUartNucleo& instance();

    DirectUartNucleo();

    int init(char *uart_dev_name, int speed);

    virtual void taskFunction();

    int send(const unsigned char *buf, size_t len);

    int recv(unsigned char *buf, size_t len);

    int set_interface_attribs(int fd, int speed);

    void exit_thread(int err_code);

  private:
    bool m_enabled;

    char m_devname[256];
    int m_uart_fd;

#ifndef WIN32
    struct termios m_savetio_remote;
#else
    unsigned char m_savetio_remote[256];
#endif

    unsigned char m_ibuf[512];
    unsigned char m_obuf[1024];

    int m_payload_byte_cnt;

    int m_uart_send_seq;
    unsigned char m_send_buf[256];
    static const int SEND_BUF_SZ = sizeof(m_send_buf);

    unsigned char m_payload_buf[4096];

    read_odo_state_t m_read_odo_state;

    int m_debug_num_points;
    short m_debug_traj_x_mm[16];
    short m_debug_traj_y_mm[16];

    static DirectUartNucleo s_instance;
  };

}
