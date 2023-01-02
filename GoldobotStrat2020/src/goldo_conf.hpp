#pragma once
#include <cstdint>
#include <cstddef>

#ifndef _u32
typedef unsigned int _u32;
#endif

namespace goldobot
{
  typedef struct _goldo_conf_calib_lidar_sample {
    double real_d;
    double meas_d;
  } goldo_conf_calib_lidar_sample_t;

  typedef struct _goldo_conf_plot_zone {
    double x_min_mm;
    double y_min_mm;
    double x_max_mm;
    double y_max_mm;
  } goldo_conf_plot_zone_t;

  typedef struct _goldo_conf_info {
    char   conf_viewer_addr_str[128];
    double conf_theta_correction_deg;
    double conf_rho_correction_factor;
    char   conf_rplidar_dev_str[128];
    _u32   conf_rplidar_baudrate;
    _u32   conf_rplidar_plot_lifetime_ms;
    bool   conf_rplidar_send_plot_enabled;
    bool   conf_rplidar_debug_plot_enabled;
    _u32   conf_rplidar_plot_nz;
    goldo_conf_plot_zone_t conf_rplidar_plot_zone[100];
    char   conf_nucleo_uart_dev_str[128];
    _u32   conf_nucleo_uart_baudrate;
    _u32   conf_zmq_port;
    _u32   conf_n_obstacles;
    char   conf_strat_file_str[128];
    char   conf_strat_file_pos_str[128];
    char   conf_strat_file_neg_str[128];
    char   conf_simul_file_str[128];
    bool   conf_dbg_log_enabled;
    _u32   conf_calib_lidar_ns;
    goldo_conf_calib_lidar_sample_t conf_calib_lidar_sample[30];
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
    static constexpr double conf_theta_correction_deg_def          = 
      30.0f; /* PR 28/05/2019 */
    static constexpr double conf_rho_correction_factor_def         = 
      1.05f;
    static constexpr _u32   conf_rplidar_plot_lifetime_ms_def      = 
      300;
    static char   conf_rplidar_dev_str_def[];
    static constexpr _u32   conf_rplidar_baudrate_def              = 
      115200;
    static constexpr bool   conf_rplidar_send_plot_enabled_def     = 
      false;
    static constexpr bool   conf_rplidar_debug_plot_enabled_def    = 
      false;
    static char   conf_nucleo_uart_dev_str_def[];
    static constexpr _u32   conf_nucleo_uart_baudrate_def          = 
      115200;
    static constexpr _u32   conf_zmq_port_def                      = 
      3101;
    static constexpr bool   conf_dbg_log_enabled_def               = 
      false;
    static constexpr _u32   conf_n_obstacles_def                   = 
      3;
    static char   conf_strat_file_str_def[];
    static char   conf_strat_file_pos_str_def[];
    static char   conf_strat_file_neg_str_def[];
    static char   conf_simul_file_str_def[]; 

    static GoldoConf s_instance;
  };
}

