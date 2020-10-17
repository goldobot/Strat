#pragma once
#include <cstdint>
#include <cstddef>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include <unistd.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "goldo_conf.hpp"
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

    int init(char* rplidar_dev, int baudrate);

    int init_viewer_sock(char *viewer_address_str);

    virtual void taskFunction();

    int send_to_viewer();

    void start_scan();

    void stop_scan();

  private:
    double rho_correction(double meas_rho);

    bool m_scanning = false;

    static const int M_NB_POINTS = 720;

    char m_rplidar_dev_str[80];

    int m_baudrate;

    float m_x[M_NB_POINTS];
    float m_y[M_NB_POINTS];

    int m_viewer_sock;
    char m_viewer_addr_str[40];
    struct sockaddr_in m_viewer_saddr;
    unsigned char m_viewer_send_buf[65500];

    unsigned int m_calib_ns;
    goldo_conf_calib_lidar_sample_t m_calib[30];

    RPlidarDriver * m_drv;

    static CommRplidar s_instance;
  };

}

