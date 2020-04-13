#pragma once
#include <cstdint>
#include <cstddef>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "goldo_thread.hpp"
#include "robot_state.hpp"


using namespace rp::standalone::rplidar;


namespace goldobot
{
  class CommRplidar : public GoldoThread
  {
  public:
    static CommRplidar& instance();

    CommRplidar();

    int init(char* rplidar_dev, double theta_correction, int baudrate);

    int init_viewer_sock(char *viewer_address_str);

    virtual void taskFunction();

    int send_to_viewer();

    void start_scan();

    void stop_scan();

  private:
    bool m_scanning = false;

    static const int M_NB_POINTS = 720;

    char m_rplidar_dev_str[80];

    double m_theta_correction;

    int m_baudrate;

    float m_x[M_NB_POINTS];
    float m_y[M_NB_POINTS];

    int m_viewer_sock;
    char m_viewer_addr_str[40];
    struct sockaddr_in m_viewer_saddr;
    unsigned char m_viewer_send_buf[65500];

    RPlidarDriver * m_drv;

    static CommRplidar s_instance;
  };

}

