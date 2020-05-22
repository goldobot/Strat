#pragma once
#include <cstdint>
#include <cstddef>

#ifndef _u32
typedef unsigned int _u32;
#endif

namespace goldobot
{
  typedef struct _goldo_conf_info {
    char   conf_viewer_addr_str[128];
    double conf_theta_correction_deg;
    char   conf_rplidar_dev_str[128];
    _u32   conf_rplidar_baudrate;
    char   conf_nucleo_uart_dev_str[128];
    _u32   conf_nucleo_uart_baudrate;
    _u32   conf_zmq_port;
    char   conf_strat_file_str[128];
    char   conf_simul_file_str[128];
  } goldo_conf_info_t;

  class GoldoConf
  {
  public:
    static GoldoConf& instance();

    GoldoConf();

    void set_default();

    int init(const char *conf_fname);

    goldo_conf_info_t& c() {return m_c;}

    void display_conf();

  private:
    goldo_conf_info_t m_c;

    char m_conf_file_name[128];

    int parse_yaml_conf(const char * yaml_fname);

    static char   conf_viewer_addr_str_def[];
    static constexpr double conf_theta_correction_deg_def  = 
      30.0f; /* PR 28/05/2019 */
    static char   conf_rplidar_dev_str_def[];
    static constexpr _u32   conf_rplidar_baudrate_def      = 
      115200;
    static char   conf_nucleo_uart_dev_str_def[];
    static constexpr _u32   conf_nucleo_uart_baudrate_def  = 
      115200;
    static constexpr _u32   conf_zmq_port_def              = 
      3101;
    static char   conf_strat_file_str_def[];
    static char   conf_simul_file_str_def[]; 

    static GoldoConf s_instance;
  };
}

